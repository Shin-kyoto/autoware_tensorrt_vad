import rosbag2_py
import cv2
import numpy as np
import argparse
from sensor_msgs.msg import CompressedImage, Image, Imu, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message, deserialize_message
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

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

    # nuscenesのメッセージをコピーし、変換行列のみを置き換え
    merged_msg = TFMessage()
    for transform in ns_tf_static_msg.transforms:
        key = (transform.header.frame_id, transform.child_frame_id)
        if key in awsim_transforms:
            # 変換行列のみをAWSIMのものに置き換え
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
