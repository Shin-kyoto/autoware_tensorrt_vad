import rosbag2_py
import cv2
import numpy as np
import argparse
from sensor_msgs.msg import CompressedImage, Image, Imu, CameraInfo, PointCloud2
from geometry_msgs.msg import AccelWithCovarianceStamped
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message, deserialize_message
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import tf2_ros
import tf2_geometry_msgs
import logging

# ロギングの設定
logging.basicConfig(
    filename='./data/log.txt',
    level=logging.INFO,
    format='%(message)s'
)

def quaternion_to_rotation_matrix(q):
    """
    クォータニオンから回転行列を計算する
    """
    x, y, z, w = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ])

def calculate_inverse_translation(translation, rotation_quat):
    """
    並進と回転クォータニオンから逆変換の並進を計算する
    """
    R = quaternion_to_rotation_matrix(rotation_quat)
    R_inv = R.T  # 回転行列の逆行列は転置
    t_inv = -R_inv @ translation
    return t_inv

def _get_target_image_size(nuscenes_bag_path):
    """
    Nuscenes rosbagの/sensing/camera/camera0/image_rect_color/compressedからターゲット画像サイズを取得する。
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=nuscenes_bag_path,
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py._storage.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        if topic_name == '/sensing/camera/camera0/image_rect_color/compressed':
            msg = deserialize_message(data, CompressedImage)
            # 圧縮画像をデコード
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                height, width = cv_image.shape[:2]
                print(f"Target image size from /sensing/camera/camera0/image_rect_color/compressed: {width}x{height}")
                return (width, height)
    raise ValueError("Error: Could not determine target image size from /sensing/camera/camera0/image_rect_color/compressed in nuscenes rosbag.")

def _load_awsim_image_raw_messages(awsim_input_bag_path, start_index=0):
    """
    Input AWSIM rosbagから/sensing/camera/image_rawメッセージをすべて読み込む。
    start_index: 開始する画像のインデックス
    """
    awsim_image_raw_msgs_list = []
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=awsim_input_bag_path,
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py._storage.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        if topic_name == '/sensing/camera/image_raw':
            msg = deserialize_message(data, Image)
            awsim_image_raw_msgs_list.append(msg)
    
    if start_index >= len(awsim_image_raw_msgs_list):
        raise ValueError(f"Start index {start_index} is out of range. Total images: {len(awsim_image_raw_msgs_list)}")
    
    print(f"Loaded {len(awsim_image_raw_msgs_list)} messages from /sensing/camera/image_raw in input AWSIM rosbag.")
    print(f"Starting from index {start_index}")
    return awsim_image_raw_msgs_list[start_index:]

def _load_awsim_tf_static_message(awsim_input_bag_path):
    """
    Input AWSIM rosbagから/tf_staticメッセージを読み込む。
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=awsim_input_bag_path,
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py._storage.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        if topic_name == '/tf_static':
            msg = deserialize_message(data, TFMessage)
            return msg
    
    raise ValueError("Error: Could not find /tf_static message in AWSIM input bag.")

def _load_ns_tf_static_message(ns_bag_path):
    """
    Nuscenes rosbagから/tf_staticメッセージを読み込む。
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=ns_bag_path,
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py._storage.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        if topic_name == '/tf_static':
            msg = deserialize_message(data, TFMessage)
            return msg
    
    raise ValueError("Error: Could not find /tf_static message in nuscenes rosbag.")

def _merge_tf_static_messages(ns_tf_static_msg, awsim_tf_static_msg):
    """
    nuscenesのtf_staticメッセージのframeとtimestampを保持しつつ、
    変換行列の値のみをAWSIMのものに置き換える。
    """
    # AWSIMの変換行列を辞書形式で保存
    awsim_transforms = {}
    for transform in awsim_tf_static_msg.transforms:
        key = (transform.header.frame_id, transform.child_frame_id)
        awsim_transforms[key] = transform.transform
    
    # Step1: awsim_transformsの結果をtf_bufferに入れる
    tf_buffer = tf2_ros.Buffer()
    for transform in awsim_tf_static_msg.transforms:
        # TransformStampedオブジェクトを作成
        transform_stamped = tf2_ros.TransformStamped()
        transform_stamped.header = transform.header
        transform_stamped.child_frame_id = transform.child_frame_id
        transform_stamped.transform = transform.transform
        # tf_bufferに静的変換として追加
        tf_buffer.set_transform_static(transform_stamped, "default_authority")
    
    # Step2: awsim_transformsの結果を入れたtf_bufferから、base_linkからcamera_linkへの変換を取得する
    try:
        # base_linkからcamera_linkへの変換を取得
        base_to_camera_transform = tf_buffer.lookup_transform(
            'base_link', 'camera_link', Time(sec=0, nanosec=0)
        )
        logging.info("=== Base Link to Camera Link Transform ===")
        logging.info(f"Translation: x={base_to_camera_transform.transform.translation.x}, y={base_to_camera_transform.transform.translation.y}, z={base_to_camera_transform.transform.translation.z}")
        logging.info(f"Rotation: x={base_to_camera_transform.transform.rotation.x}, y={base_to_camera_transform.transform.rotation.y}, z={base_to_camera_transform.transform.rotation.z}, w={base_to_camera_transform.transform.rotation.w}")
        logging.info("==========================================")
    except Exception as e:
        logging.error(f"Error getting transform from base_link to camera_link: {e}")
        base_to_camera_transform = None
    # nuscenesのメッセージをコピーし、変換行列のみを置き換え
    merged_msg = TFMessage()
    for transform in ns_tf_static_msg.transforms:
        # Step3: base_linkからcamera_linkへの変換を、transform.header.frame_id == 'base_link' and transform.child_frame_id == 'camera0/camera_optical_link'に入れる
        if transform.child_frame_id == 'camera0/camera_optical_link' and transform.header.frame_id == 'base_link':
            if base_to_camera_transform is not None:
                # 目標の並進と回転（tf2_echoで表示される値）
                target_translation = np.array([0.0, 0.9, -0.4])
                target_rotation_quat = np.array([0.487, -0.486, 0.507, 0.519])
                
                # 逆変換の並進を計算
                # tf2_echo camera0/camera_optical_link base_link で表示される並進を計算
                inverse_translation = calculate_inverse_translation(target_translation, target_rotation_quat)
                
                # 並進部分を計算された逆変換値で設定
                transform.transform.translation.x = inverse_translation[0]
                transform.transform.translation.y = inverse_translation[1]
                transform.transform.translation.z = inverse_translation[2]
                
                # 回転部分を指定された値で強制上書き
                # 目標: tf2_echo camera0/camera_optical_link base_link で以下が表示されるようにする
                # Rotation: in Quaternion [0.487, -0.486, 0.507, 0.519]
                # Rotation: in RPY (radian) [0.238, -1.515, 1.323]
                # Rotation: in RPY (degree) [13.629, -86.822, 75.792]
                # 
                # base_link -> camera0/camera_optical_link の変換を設定するが、
                # tf2_echo は逆変換を表示するため、逆変換の回転を正しく計算
                # クォータニオンの逆変換: [x, y, z, w] -> [-x, -y, -z, w]
                transform.transform.rotation.x = -0.487
                transform.transform.rotation.y = 0.486
                transform.transform.rotation.z = -0.507
                transform.transform.rotation.w = 0.519
                
                logging.info("=== TF Static Transformation Matrix (Updated with Fixed Values) ===")
                logging.info(f"Frame: {transform.header.frame_id} -> {transform.child_frame_id}")
                logging.info(f"Translation (Calculated): x={transform.transform.translation.x}, y={transform.transform.translation.y}, z={transform.transform.translation.z}")
                logging.info(f"Rotation (Fixed): x={transform.transform.rotation.x}, y={transform.transform.rotation.y}, z={transform.transform.rotation.z}, w={transform.transform.rotation.w}")
                logging.info("Expected tf2_echo output:")
                logging.info("Translation: [0, -0.7, -0.8]")
                logging.info("Rotation: in Quaternion [0.487, -0.486, 0.507, 0.519]")
                logging.info("Rotation: in RPY (radian) [0.238, -1.515, 1.323]")
                logging.info("Rotation: in RPY (degree) [13.629, -86.822, 75.792]")
                logging.info("================================================================")
        else:
            # 他の変換については、直接マッチするものがあれば置き換え
            key = (transform.header.frame_id, transform.child_frame_id)
            if key in awsim_transforms:
                transform.transform = awsim_transforms[key]
        
        merged_msg.transforms.append(transform)

    return merged_msg

def replace_camera0_image_sequentially(original_image_msg, awsim_image_raw_iterator, target_image_size, bridge):
    """
    /sensing/camera/camera0/image_rect_color/compressedの画像をAWSIMの/sensing/camera/image_rawの画像で順番に置き換える。
    """
    try:
        # イテレータから次のAWSIM画像メッセージを取得
        awsim_raw_image_msg = next(awsim_image_raw_iterator)

        # AWSIM画像をOpenCV形式に変換
        cv_image = bridge.imgmsg_to_cv2(awsim_raw_image_msg, desired_encoding="bgr8")
        # ターゲットサイズにリサイズ
        resized_image = cv2.resize(cv_image, target_image_size, interpolation=cv2.INTER_AREA)
        
        # 画像をJPEG形式で圧縮
        _, compressed_data = cv2.imencode('.jpg', resized_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        
        # 新しいCompressedImageメッセージを作成
        new_image_msg = CompressedImage()
        new_image_msg.header = original_image_msg.header
        new_image_msg.format = 'jpeg'
        new_image_msg.data = compressed_data.tobytes()
        
        return new_image_msg
    except StopIteration:
        print(f"Warning: Ran out of /sensing/camera/image_raw messages from AWSIM input bag. Using original /sensing/camera/camera0/image_rect_color/compressed message for current time {original_image_msg.header.stamp}.")
        return original_image_msg
    except Exception as e:
        print(f"Error processing image for /sensing/camera/camera0/image_rect_color/compressed at time {original_image_msg.header.stamp}: {e}. Returning original message.")
        return original_image_msg

def create_blackout_image(original_image_msg, target_image_size, bridge):
    """
    指定されたサイズの黒塗り画像を生成する。
    """
    try:
        # 黒画像を生成
        black_image = np.zeros((target_image_size[1], target_image_size[0], 3), dtype=np.uint8)
        
        # 画像をJPEG形式で圧縮
        _, compressed_data = cv2.imencode('.jpg', black_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
        
        # 新しいCompressedImageメッセージを作成
        new_image_msg = CompressedImage()
        new_image_msg.header = original_image_msg.header
        new_image_msg.format = 'jpeg'
        new_image_msg.data = compressed_data.tobytes()
        
        return new_image_msg
    except Exception as e:
        print(f"Error creating blackout image: {e}. Returning original message.")
        return original_image_msg

def write_to_rosbag(writer, topic: str, msg, timestamp: Time):
    """
    メッセージをROSバッグに書き込む
    
    Args:
        writer: ROSバッグライター
        topic: トピック名
        msg: 書き込むメッセージ
        timestamp: メッセージのタイムスタンプ
    """
    # タイムスタンプをナノ秒に変換（小数点以下の精度を保持）
    ros_timestamp = int(timestamp.sec * 1e9) + timestamp.nanosec
    writer.write(topic, serialize_message(msg), ros_timestamp)

def _load_awsim_camera0_info(awsim_input_bag_path):
    """
    AWSIM rosbagから/sensing/camera/camera0/camera_infoメッセージを取得する。
    """
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=awsim_input_bag_path,
        storage_id="sqlite3"
    )
    converter_options = rosbag2_py._storage.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    while reader.has_next():
        topic_name, data, timestamp_ns = reader.read_next()
        if topic_name == '/sensing/camera/camera_info':
            msg = deserialize_message(data, CameraInfo)
            return msg
    raise ValueError("Error: Could not find /sensing/camera/camera_info in AWSIM input bag.")

def _overwrite_camera_info_matrix(ns_camera_info_msg, awsim_camera_info_msg):
    """
    nuScenesのcamera_infoのframeやtimestampはそのまま、行列部分のみAWSIMの値で上書きする。
    """
    ns_camera_info_msg.k = awsim_camera_info_msg.k
    ns_camera_info_msg.p = awsim_camera_info_msg.p
    ns_camera_info_msg.d = awsim_camera_info_msg.d
    ns_camera_info_msg.r = awsim_camera_info_msg.r
    ns_camera_info_msg.distortion_model = awsim_camera_info_msg.distortion_model
    ns_camera_info_msg.height = awsim_camera_info_msg.height
    ns_camera_info_msg.width = awsim_camera_info_msg.width

    # camera_infoの座標変換行列をログ出力
    logging.info("\n=== Camera Info Matrix ===")
    logging.info("Intrinsic Matrix (K):")
    logging.info(f"[[{ns_camera_info_msg.k[0]}, {ns_camera_info_msg.k[1]}, {ns_camera_info_msg.k[2]}],")
    logging.info(f" [{ns_camera_info_msg.k[3]}, {ns_camera_info_msg.k[4]}, {ns_camera_info_msg.k[5]}],")
    logging.info(f" [{ns_camera_info_msg.k[6]}, {ns_camera_info_msg.k[7]}, {ns_camera_info_msg.k[8]}]]")
    
    logging.info("\nProjection Matrix (P):")
    logging.info(f"[[{ns_camera_info_msg.p[0]}, {ns_camera_info_msg.p[1]}, {ns_camera_info_msg.p[2]}, {ns_camera_info_msg.p[3]}],")
    logging.info(f" [{ns_camera_info_msg.p[4]}, {ns_camera_info_msg.p[5]}, {ns_camera_info_msg.p[6]}, {ns_camera_info_msg.p[7]}],")
    logging.info(f" [{ns_camera_info_msg.p[8]}, {ns_camera_info_msg.p[9]}, {ns_camera_info_msg.p[10]}, {ns_camera_info_msg.p[11]}]]")
    
    logging.info("\nDistortion Coefficients (D):")
    logging.info(f"[{ns_camera_info_msg.d[0]}, {ns_camera_info_msg.d[1]}, {ns_camera_info_msg.d[2]}, {ns_camera_info_msg.d[3]}, {ns_camera_info_msg.d[4]}]")
    
    logging.info("\nRectification Matrix (R):")
    logging.info(f"[[{ns_camera_info_msg.r[0]}, {ns_camera_info_msg.r[1]}, {ns_camera_info_msg.r[2]}],")
    logging.info(f" [{ns_camera_info_msg.r[3]}, {ns_camera_info_msg.r[4]}, {ns_camera_info_msg.r[5]}],")
    logging.info(f" [{ns_camera_info_msg.r[6]}, {ns_camera_info_msg.r[7]}, {ns_camera_info_msg.r[8]}]]")
    logging.info("========================\n")

    return ns_camera_info_msg

def process_rosbags(nuscenes_rosbag_path, input_awsim_rosbag_path, output_awsim_rosbag_path, start_index=0):
    """
    Nuscenes rosbagの画像をinput-awsim-rosbagの画像で順番に置き換え、他のカメラ画像を黒塗りにして保存する
    start_index: AWSIMのrosbagから開始する画像のインデックス
    """
    bridge = CvBridge()

    try:
        # ターゲット画像サイズとAWSIM画像を事前に読み込む
        target_image_size = _get_target_image_size(nuscenes_rosbag_path)
        awsim_image_raw_msgs_list = _load_awsim_image_raw_messages(input_awsim_rosbag_path, start_index)
        awsim_tf_static_msg = _load_awsim_tf_static_message(input_awsim_rosbag_path)
        ns_tf_static_msg = _load_ns_tf_static_message(nuscenes_rosbag_path)
        awsim_camera0_info_msg = _load_awsim_camera0_info(input_awsim_rosbag_path)
        
        # tf_staticメッセージをマージ
        merged_tf_static_msg = _merge_tf_static_messages(ns_tf_static_msg, awsim_tf_static_msg)
        
        # AWSIM画像リストをイテレータに変換
        awsim_image_raw_iterator = iter(awsim_image_raw_msgs_list)

        print(f"Loading nuscenes rosbag: {nuscenes_rosbag_path}")
        print(f"Loading input awsim rosbag: {input_awsim_rosbag_path}")

        # ROSバッグの初期化
        writer = rosbag2_py.SequentialWriter()
        
        # ストレージオプションの設定
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=output_awsim_rosbag_path,
            storage_id="sqlite3"
        )
        
        # 変換オプションの設定
        converter_options = rosbag2_py._storage.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
        
        # バッグファイルのオープン
        writer.open(storage_options, converter_options)

        # メタデータの作成と登録
        topic_types = [
            # カメラ画像
            ("/sensing/camera/camera0/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage"),
            ("/sensing/camera/camera1/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage"),
            ("/sensing/camera/camera2/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage"),
            ("/sensing/camera/camera3/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage"),
            ("/sensing/camera/camera4/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage"),
            ("/sensing/camera/camera5/image_rect_color/compressed", "sensor_msgs/msg/CompressedImage"),
            # カメラ情報
            ("/sensing/camera/camera0/camera_info", "sensor_msgs/msg/CameraInfo"),
            ("/sensing/camera/camera1/camera_info", "sensor_msgs/msg/CameraInfo"),
            ("/sensing/camera/camera2/camera_info", "sensor_msgs/msg/CameraInfo"),
            ("/sensing/camera/camera3/camera_info", "sensor_msgs/msg/CameraInfo"),
            ("/sensing/camera/camera4/camera_info", "sensor_msgs/msg/CameraInfo"),
            ("/sensing/camera/camera5/camera_info", "sensor_msgs/msg/CameraInfo"),
            # その他のセンサー
            ("/sensing/imu/tamagawa/imu_raw", "sensor_msgs/msg/Imu"),
            ("/sensing/lidar/concatenated/pointcloud", "sensor_msgs/msg/PointCloud2"),
            # ローカライゼーション
            ("/localization/kinematic_state", "nav_msgs/msg/Odometry"),
            ("/localization/acceleration", "geometry_msgs/msg/AccelWithCovarianceStamped"),
            # TF
            ("/tf_static", "tf2_msgs/msg/TFMessage"),
        ]
        
        # トピックの情報を登録
        for topic_name, topic_type in topic_types:
            topic_info = rosbag2_py._storage.TopicMetadata(
                name=topic_name,
                type=topic_type,
                serialization_format="cdr"
            )
            writer.create_topic(topic_info)

        # Nuscenes rosbagの読み込み
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py._storage.StorageOptions(
            uri=nuscenes_rosbag_path,
            storage_id="sqlite3"
        )
        converter_options = rosbag2_py._storage.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr"
        )
        reader.open(storage_options, converter_options)

        # Nuscenes rosbagの各メッセージを処理
        while reader.has_next():
            topic_name, data, timestamp_ns = reader.read_next()
            timestamp = Time(sec=int(timestamp_ns // 1e9), nanosec=int(timestamp_ns % 1e9))

            if topic_name.startswith('/sensing/camera/camera') and topic_name.endswith('/image_rect_color/compressed'):
                msg = deserialize_message(data, CompressedImage)
                if topic_name == '/sensing/camera/camera0/image_rect_color/compressed':
                    # AWSIM画像を順番に取得し、置き換える
                    new_msg = replace_camera0_image_sequentially(msg, awsim_image_raw_iterator, target_image_size, bridge)
                    write_to_rosbag(writer, topic_name, new_msg, timestamp)
                else:
                    # 他のカメラ画像を黒塗り化
                    new_msg = create_blackout_image(msg, target_image_size, bridge)
                    write_to_rosbag(writer, topic_name, new_msg, timestamp)
            elif topic_name == '/sensing/camera/camera0/camera_info':
                msg = deserialize_message(data, CameraInfo)
                new_msg = _overwrite_camera_info_matrix(msg, awsim_camera0_info_msg)
                write_to_rosbag(writer, topic_name, new_msg, timestamp)
            elif topic_name.endswith('/camera_info'):
                msg = deserialize_message(data, CameraInfo)
                write_to_rosbag(writer, topic_name, msg, timestamp)
            elif topic_name == '/sensing/imu/tamagawa/imu_raw':
                msg = deserialize_message(data, Imu)
                write_to_rosbag(writer, topic_name, msg, timestamp)
            elif topic_name == '/sensing/lidar/concatenated/pointcloud':
                msg = deserialize_message(data, PointCloud2)
                write_to_rosbag(writer, topic_name, msg, timestamp)
            elif topic_name == '/localization/kinematic_state':
                msg = deserialize_message(data, Odometry)
                write_to_rosbag(writer, topic_name, msg, timestamp)
            elif topic_name == '/tf_static':
                # マージしたtf_staticメッセージを使用
                write_to_rosbag(writer, topic_name, merged_tf_static_msg, timestamp)
            elif topic_name == '/localization/acceleration':
                msg = deserialize_message(data, AccelWithCovarianceStamped)
                write_to_rosbag(writer, topic_name, msg, timestamp)
            else:
                # その他のメッセージはそのまま書き込み
                msg = deserialize_message(data, get_message_type(topic_name))
                write_to_rosbag(writer, topic_name, msg, timestamp)

        print(f"Successfully processed and saved to {output_awsim_rosbag_path}")

    except ValueError as ve:
        print(f"Configuration Error: {ve}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Replace camera0 image in nuscenes rosbag with input awsim rosbag image sequentially and blackout other camera images.")
    parser.add_argument('--nuscenes-rosbag', type=str, required=True,
                        help="Path to the nuscenes rosbag file.")
    parser.add_argument('--input-awsim-rosbag', type=str, required=True,
                        help="Path to the input AWSIM rosbag file containing /sensing/camera/image_raw.")
    parser.add_argument('--output-awsim-rosbag', type=str, required=True,
                        help="Path to the output AWSIM rosbag file.")
    parser.add_argument('--start-index', type=int, default=0,
                        help="Index of the first image to use from AWSIM rosbag (default: 0)")

    args = parser.parse_args()

    process_rosbags(args.nuscenes_rosbag, args.input_awsim_rosbag, args.output_awsim_rosbag, args.start_index)
