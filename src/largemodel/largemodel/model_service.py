import os
import cv2
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String,Bool
from utils import large_model_interface
from utils.tools_manager import ToolsManager
from ament_index_python.packages import get_package_share_directory
import threading
import pygame
import numpy as np
import atexit
from interfaces.srv import TextToImage

class LargeModelService(Node):
    def __init__(self):
        super().__init__('LargeModelService')

        self.init_param_config()

        # Dependencies first: initialize all tools and services. / 依赖前置：先初始化所有工具和服务。
        self.tools_manager = ToolsManager(self)
        from utils.ai_agent import AIAgent
        self.ai_agent = AIAgent(self, self.tools_manager)
        from utils.mcp_server import MCPServer
        self.mcp_server = MCPServer(self, self.tools_manager)

        # Then initialize the large models that depend on them. / 再初始化依赖它们的大模型。
        self.init_largemodel()

        # Finally, initialize other modules. / 最后初始化其他模块。
        self.init_ros_comunication()

        # Add initialization for camera-related attributes. / 添加摄像头相关属性初始化。
        self.camera_initialized = False
        self.camera_warning_issued = False
        self.cap = None

        # Initialize the audio player. / 初始化音频播放器。
        pygame.mixer.init()
        self.current_thread = None
        self.stop_event = threading.Event()

        # Used to save and trace Ollama's messages in multi-turn conversations. / 用于在多轮对话中保存和追溯Ollama的message。
        self.message_file_path = os.path.join(self.pkg_path, "resources_file", "conversation_message.json")
        self.init_message_file()
        atexit.register(self.cleanup_message_file)

        # Create a GUI update timer to resolve the OpenCV window freezing issue. / 创建GUI更新定时器，解决OpenCV弹窗卡死问题。
        self.gui_update_timer = self.create_timer(0.033, self._gui_update_callback)  # Approximately 30Hz. / 约30Hz。

        # Log information. / 打印日志。
        self.get_logger().info('LargeModelService node Initialization completed...')
        
        self.init_camera()
        self._ensure_camera_is_ready()

    def _gui_update_callback(self):
        """GUI update callback function to handle OpenCV window events. / GUI更新回调函数，处理OpenCV窗口事件。"""
        try:
            cv2.waitKey(1)
        except Exception as e:
            pass

    def init_param_config(self):
        """Initializes ROS parameters and file paths. / 初始化ROS参数和文件路径。"""
        src_path = os.path.expanduser("~/yahboom_ws/src/largemodel")
        if os.path.exists(src_path):
            self.pkg_path = src_path
            self.get_logger().info(f"Using development path: {self.pkg_path}")
        else:
            self.pkg_path = get_package_share_directory('largemodel')
            self.get_logger().info(f"Using install path: {self.pkg_path}")
        # Declare parameters. / 参数声明。
        self.declare_parameter('language', 'zh')
        self.declare_parameter('text_chat_mode', False)
        self.declare_parameter('llm_platform', 'ollama')
        self.declare_parameter('useolinetts', False)
        self.declare_parameter("regional_setting", "China")

        # Get parameters from the parameter server. / 获取参数服务器参数。
        self.language = self.get_parameter('language').get_parameter_value().string_value 
        self.text_chat_mode = self.get_parameter('text_chat_mode').get_parameter_value().bool_value 
        self.useolinetts = self.get_parameter('useolinetts').get_parameter_value().bool_value
        self.llm_platform = self.get_parameter('llm_platform').get_parameter_value().string_value
        self.regional_setting = (self.get_parameter("regional_setting").get_parameter_value().string_value)

        # Set TTS output path. / 设置TTS输出路径。
        

    def init_largemodel(self):
        """Initializes the large language model client based on configuration. / 根据配置初始化大语言模型客户端。"""
        # Create a model interface client and pass the platform name and logger. / 创建模型接口客户端,并传入平台名称和logger。
        self.model_client = large_model_interface.model_interface(
            llm_platform=self.llm_platform,
            logger=self.get_logger(),
            mcp_server=self.mcp_server
        )
        # Call LLM initialization. / 调用LLM初始化。
        self.model_client.init_llm()
        # Initialize the language for the large model interface. / 初始化大模型接口的语言。
        self.model_client.init_language(self.language)
        self.model_client.init_messages()
        self.get_logger().info(f'Using LLM platform: {self.llm_platform}')

    def init_ros_comunication(self):
        """Initializes all ROS2 publishers and subscribers. / 初始化所有ROS2的发布者和订阅者。"""
        # ASR topic subscriber. / asr话题订阅者。
        self.asrsub = self.create_subscription(String,'asr', self.asr_callback,1)
        # Create a text interaction publisher. / 创建文字交互发布者。
        self.text_pub = self.create_publisher(String, "text_response", 1)
        # Create a TTS publisher. / 创建TTS发布者。
        self.TTS_publisher = self.create_publisher(String, "tts_topic", 5)
        # Initialize the TTS system. / 初始化TTS系统。
        self.system_sound_init()
        # Create a wake-up subscriber. / 创建唤醒订阅者。
        self.wakeup_sub = self.create_subscription(Bool, 'wakeup', self.wakeup_callback, 5)

    def init_message_file(self):
        """Initialize the message file. / 初始化message文件。"""
        try:
            # Ensure the directory exists. / 确保目录存在。
            os.makedirs(os.path.dirname(self.message_file_path), exist_ok=True)
            
            # If the file does not exist or is empty, create an empty message array. / 如果文件不存在或为空，创建空的message数组。
            if not os.path.exists(self.message_file_path):
                with open(self.message_file_path, 'w', encoding='utf-8') as f:
                    json.dump([], f, ensure_ascii=False, indent=2)  # Ensure it's an empty array. / 确保是空数组。
            else:
                # Check the format of the existing file. / 检查现有文件格式。
                try:
                    with open(self.message_file_path, 'r', encoding='utf-8') as f:
                        content = f.read().strip()
                        if content:
                            json.loads(content)  # Verify JSON format. / 验证JSON格式。
                        else:
                            # If the file is empty, write an empty array. / 文件为空，写入空数组。
                            with open(self.message_file_path, 'w', encoding='utf-8') as f:
                                json.dump([], f, ensure_ascii=False, indent=2)
                except json.JSONDecodeError:
                    # If the file format is incorrect, reinitialize. / 文件格式错误，重新初始化。
                    self.get_logger().warning("Message file format error, reinitializing...")
                    with open(self.message_file_path, 'w', encoding='utf-8') as f:
                        json.dump([], f, ensure_ascii=False, indent=2)
                    
            self.get_logger().info(f"Message file initialized at: {self.message_file_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize message file: {e}")

    def save_message_to_file(self, messages):
        """Save messages to a file. / 保存message到文件。"""
        try:
            with open(self.message_file_path, 'w', encoding='utf-8') as f:
                messages_to_save = messages if messages is not None else []
                
                # Ensure the uniqueness and existence of the system prompt. / 确保 system prompt 的唯一性和存在性。
                # 1. Get the correct initial message list containing the system prompt from memory. / 1. 从内存中获取包含 system prompt 的、正确的初始消息列表。
                initial_template = self.model_client.messages.copy()
                
                # 2. From the messages to be saved, filter out only the user and assistant conversation history. / 2. 从待保存的消息中，只筛选出 user 和 assistant 的对话历史。
                history_only = [m for m in messages_to_save if m.get('role') in ['user', 'assistant']]
                
                # 3. Merge the correct initial template with the clean conversation history. / 3. 将正确的初始模板和纯净的对话历史合并。
                final_messages = initial_template + history_only

                # Limit the size of messages to prevent the file from becoming too large. / 限制messages大小，防止文件过大。
                if len(final_messages) > 50:
                    # Truncate only the user and assistant messages. / 只截断 user 和 assistant 的消息。
                    recent_history = [m for m in final_messages if m.get('role') in ['user', 'assistant']][-40:]
                    # Re-merge with the initial template and the truncated history. / 重新用初始模板和截断后的历史合并。
                    final_messages = initial_template + recent_history
                    self.get_logger().info("Messages truncated before saving to file")
                
                json.dump(final_messages, f, ensure_ascii=False, indent=2)
                    
            self.get_logger().debug(f"Messages saved to file, size: {len(final_messages)}")
        except Exception as e:
            self.get_logger().error(f"Failed to save messages to file: {e}")



    def load_message_from_file(self):
        """Load messages from a file. / 从文件加载messages。"""
        try:
            if os.path.exists(self.message_file_path):
                with open(self.message_file_path, 'r', encoding='utf-8') as f:
                    content = f.read().strip()
                    if not content:
                        return []  # Return empty list for empty file
                    messages = json.loads(content)
                    
                # Check and limit the number of messages. / 检查并限制消息数量。
                if isinstance(messages, list) and len(messages) > 100:
                    system_msgs = [msg for msg in messages if msg.get('role') == 'system']
                    recent_msgs = [msg for msg in messages if msg.get('role') != 'system'][-80:]
                    messages = system_msgs + recent_msgs
                    self.save_message_to_file(messages)  # Save the truncated messages. / 保存截断后的messages。
                    self.get_logger().info("Messages truncated after loading from file")
                    
                self.get_logger().debug(f"Messages loaded from file, count: {len(messages) if messages else 0}")
                return messages
            else:
                return []  # File doesn't exist, return empty list
        except Exception as e:
            self.get_logger().error(f"Failed to load messages from file: {e}")
            self.clear_message_file()  # Clear corrupted file
            return []  # Return empty list

    def cleanup_message_file(self):
        """Clean up the message file. / 清理message文件。"""
        try:
            if os.path.exists(self.message_file_path):
                os.remove(self.message_file_path)
                print(f"Message file cleaned up: {self.message_file_path}")
        except Exception as e:
            print(f"Failed to cleanup message file: {e}")

    def clear_message_file(self):
        """Clear the message content. / 清空message内容。"""
        self.save_message_to_file(None)
        self.get_logger().info("Message file cleared")



    def init_camera(self):
        """Initialize camera configuration / 初始化摄像头配置"""
        # Camera configuration is read from a yaml file. / 摄像头配置从yaml文件读取。
        self.declare_parameter('camera_width', 640)
        self.declare_parameter('camera_height', 480)
        self.declare_parameter('camera_type', 'usb')  # 添加摄像头类型参数: 'csi', 'usb'
        self.declare_parameter('mjpeg_stream_url', 'http://172.17.0.1:8080/camera.mjpg')
        
        self.camera_width = self.get_parameter('camera_width').get_parameter_value().integer_value
        self.camera_height = self.get_parameter('camera_height').get_parameter_value().integer_value
        self.camera_type_param = self.get_parameter('camera_type').get_parameter_value().string_value.lower()
        self.mjpeg_stream_url = self.get_parameter('mjpeg_stream_url').get_parameter_value().string_value
        
        # Initialize camera variables. / 初始化摄像头变量。
        self.cap = None
        self.camera_initialized = False
        self.last_frame = None
        
        self.get_logger().info(f'Camera system configured (will initialize on demand). Camera type: {self.camera_type_param}')

    def _ensure_camera_is_ready(self) -> bool:
        """Ensures the camera is initialized before use, trying to initialize it if not. / 确保摄像头在使用前已初始化，如果未初始化则尝试初始化。"""
        if self.camera_initialized:
            return True

        # Attempt to initialize the device. / 尝试初始化设备。
        if self.init_camera_device():
            return True
        else:
            # If initialization fails, check if a warning has already been issued. / 初始化失败，检查是否已经发过警告。
            if not self.camera_warning_issued:
                self.get_logger().error("Camera initialization failed! Please check if the camera is connected correctly. Tools that require the camera will not be available.")
                self.camera_warning_issued = True 
            return False

    def init_camera_device(self):
        """
        Initialize camera based on configuration parameter.
        根据配置参数初始化摄像头。
        """
        self.get_logger().info(f"Attempting to initialize camera type: {self.camera_type_param}")
        
        if self.camera_type_param == 'usb':
            if self._init_usb_camera():
                return True
            else:
                self.get_logger().error("Failed to initialize specified USB camera.")
                return False
        elif self.camera_type_param == 'csi':
            if self._init_csi_camera():
                return True
            else:
                self.get_logger().error("Failed to initialize specified CSI camera.")
                return False
        else:
            self.get_logger().error(f"Invalid camera type specified: {self.camera_type_param}. Valid options are: 'csi', 'usb'")
            return False

    def _init_csi_camera(self):
        """Initialize CSI camera by connecting to an MJPEG stream."""
        try:
            self.get_logger().info(f"Attempting to connect to MJPEG stream for CSI camera at: {self.mjpeg_stream_url}")
            
            # Create a VideoCapture object from the MJPEG stream URL
            cap = cv2.VideoCapture(self.mjpeg_stream_url)
            
            # Check if it opened successfully
            if cap.isOpened():
                # Try reading a frame to verify the stream is working
                ret, frame = cap.read()
                if ret:
                    self.get_logger().info("MJPEG stream for CSI Camera connected successfully.")
                    self.cap = cap
                    self.camera_initialized = True
                    self.camera_type = "csi_mjpeg"  # Indicate the source type
                    return True
                else:
                    # If unable to read a frame, release resources
                    cap.release()
                    self.get_logger().warn("Connected to MJPEG stream but failed to read frame.")
            else:
                self.get_logger().warn(f"Failed to open MJPEG stream at {self.mjpeg_stream_url}.")
                
            # If all attempts fail
            self.get_logger().error("Failed to initialize CSI camera from MJPEG stream.")
            if 'cap' in locals() and cap is not None:
                cap.release()
            self.cap = None
            return False
            
        except Exception as e:
            self.get_logger().error(f"Exception during CSI camera (MJPEG stream) initialization: {e}")
            if 'cap' in locals() and cap is not None:
                try:
                    cap.release()
                except:
                    pass
            self.cap = None
            return False
            
    def _detect_usb_camera_device(self):
        """
        Detect USB camera device by checking /sys/class/video4linux entries.
        通过检查/sys/class/video4linux条目来检测USB摄像头设备。
        
        :return: device ID or None
        """
        try:
            import subprocess
            import re
            
            # 获取所有video设备的信息/Get information about all video devices
            result = subprocess.run(['ls', '/sys/class/video4linux'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self.get_logger().warn("Failed to list video4linux devices")
                return None
                
            video_devices = result.stdout.strip().split('\n')
            if not video_devices or video_devices == ['']:
                self.get_logger().warn("No video4linux devices found")
                return None
                
            # 按设备号排序，确保顺序一致/Sort by device number to ensure consistent order
            video_devices.sort(key=lambda x: int(re.search(r'video(\d+)', x).group(1)) if re.search(r'video(\d+)', x) else 0)
            
            # 对每个设备检查其链接信息，寻找USB摄像头/Check the connection information for each device and look for USB cameras
            for device in video_devices:
                if not device.startswith('video'):
                    continue
                    
                try:
                    # 使用readlink获取设备的真实路径/Use readlink to get the real path of the device
                    cmd = ['readlink', '-f', f'/sys/class/video4linux/{device}']
                    result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
                    
                    if result.returncode == 0:
                        real_path = result.stdout.strip()
                        self.get_logger().debug(f"Device {device} real path: {real_path}")
                        
                        # USB摄像头通常与usb相关，且不是CSI摄像头/USB cameras are usually related to USB and are not CSI cameras
                        if 'usb' in real_path.lower() and 'tegra' not in real_path.lower() and 'capture-vi' not in real_path.lower():
                            # 提取设备号/Extract device number
                            device_number = re.search(r'video(\d+)', device)
                            if device_number:
                                device_id = int(device_number.group(1))
                                self.get_logger().info(f"Detected USB camera at device: /dev/{device} (ID: {device_id})")
                                return device_id
                except Exception as e:
                    self.get_logger().debug(f"Failed to check device {device}: {e}")
                    continue
                    
            # 如果没有找到明确的USB设备，返回第一个非CSI设备/If no specific USB device is found, returns the first non-CSI device
            for device in video_devices:
                if not device.startswith('video'):
                    continue
                    
                try:
                    cmd = ['readlink', '-f', f'/sys/class/video4linux/{device}']
                    result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
                    
                    if result.returncode == 0:
                        real_path = result.stdout.strip()
                        # 跳过CSI设备/Skip CSI devices
                        if 'tegra' not in real_path.lower() and 'capture-vi' not in real_path.lower():
                            device_number = re.search(r'video(\d+)', device)
                            if device_number:
                                device_id = int(device_number.group(1))
                                self.get_logger().info(f"Using first non-CSI device as USB camera: /dev/{device} (ID: {device_id})")
                                return device_id
                except Exception as e:
                    continue
                        
            # 如果还是没有找到，返回None/If still not found, return None
            self.get_logger().warn("No USB camera device found")
            return None
            
        except Exception as e:
            self.get_logger().warn(f"Failed to detect USB camera device: {e}")
            
        return None

    def _init_usb_camera(self):
        """Initialize USB camera using OpenCV."""
        try:
            self.get_logger().info("Attempting to initialize USB camera with auto-detected device")
            
            # 自动检测USB摄像头设备ID/Automatically detect USB camera device ID
            usb_device_id = self._detect_usb_camera_device()
            if usb_device_id is None:
                self.get_logger().warn("Failed to detect USB camera device, using default device 0")
                usb_device_id = 0
            else:
                self.get_logger().info(f"Using detected USB camera device ID: {usb_device_id}")
            
            # 使用检测到的设备ID初始化USB摄像头/Initialize the USB camera using the detected device ID
            cap = cv2.VideoCapture(usb_device_id)
            
            # 检查是否成功打开/Check if it is opened successfully
            if cap.isOpened():
                # 设置分辨率/Set the resolution
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_height)
                
                # 尝试读取一帧来验证摄像头是否真正工作/Try reading a frame to verify that the camera is actually working
                ret, frame = cap.read()
                if ret:
                    self.get_logger().info(f"USB Camera initialized successfully at {self.camera_width}x{self.camera_height}.")
                    self.cap = cap
                    self.camera_initialized = True
                    self.camera_type = "usb"
                    return True
                else:
                    # 如果无法读取帧，释放资源/If the frame cannot be read, release the resources
                    cap.release()
                    self.get_logger().warn("USB Camera opened but failed to read frame.")
            else:
                self.get_logger().warn("Failed to open USB camera.")
                
            # 如果所有尝试都失败/If all else fails
            self.get_logger().error("Failed to initialize USB camera.")
            if cap:
                cap.release()
            self.cap = None
            return False
            
        except Exception as e:
            self.get_logger().error(f"Exception during USB camera initialization: {e}")
            if cap:
                try:
                    cap.release()
                except:
                    pass
            self.cap = None
            return False

    # TTS initialization. / TTS初始化。
    def system_sound_init(self):
        """Initialize TTS system"""
        if self.regional_setting == "China":  # 如果是中国地区
            if self.useolinetts:
                model_type = "oline"
                self.tts_out_path = os.path.join(self.pkg_path, "resources_file", "tts_output.mp3")
            else:
                model_type = "local"
                self.tts_out_path = os.path.join(self.pkg_path, "resources_file", "tts_output.wav")
        elif self.regional_setting == "international":  # 如果是国际地区
            model_type = "XUNFEI_FOR_INTERNATIONAL"
            self.tts_out_path = os.path.join(self.pkg_path, "resources_file", "XUNFEI_TTS.mp3")
        else:
            while True:
                self.get_logger().info()(
                    'Please check the regional_setting parameter in yahboom.yaml file, it should be either "China" or "international".'
                )
                time.sleep(1)
        self.model_client.tts_model_init(model_type, self.language)
        self.get_logger().info(f'TTS initialized with {model_type} model')

    # Wake-up callback. / 唤醒回调。
    def wakeup_callback(self, msg):
        """Handle wake-up signals"""
        if msg.data:
            self.get_logger().info('Received wake-up signal, attempting to stop audio.')
            if pygame.mixer.music.get_busy():
                pygame.mixer.music.stop()
                self.get_logger().info('pygame.mixer.music stopped directly.')
            
            if self.current_thread and self.current_thread.is_alive():
                self.stop_event.set()

    def asr_callback(self,msg):
        """Callback function for handling ASR messages. / 处理ASR消息的回调函数。"""
        # 1. Load history from file. / 1. 从文件加载历史记录。
        messages_to_use = self.load_message_from_file()

        # 2. Check if the history is valid. If it's empty or doesn't contain a system prompt,
        #    it indicates a new conversation, so initialize with the template from memory.
        #/ 2. 检查历史记录是否有效。如果为空或不包含system prompt，
        #    则说明是一次全新的对话，使用内存中的模板进行初始化。
        if not messages_to_use or not any(d.get('role') == 'system' for d in messages_to_use):
            self.get_logger().info("No valid history in file, starting a new conversation with template.")
            messages_to_use = self.model_client.messages.copy()

        # Call the new unified interface. infer_with_text will append the current user input (msg.data).
        #/ 调用新的统一接口。infer_with_text会负责将当前用户输入(msg.data)追加进去
        result = self.model_client.infer_with_text(msg.data, message=messages_to_use)

        self.process_model_result(result)

    def process_model_result(self, result, from_seewhat=False):
        """Process the result returned by the model. / 处理模型返回的结果。"""
        response_text = ""
        tools_list = []

        if isinstance(result, dict):
            new_messages = result.get('messages')
            if new_messages:
                self.save_message_to_file(new_messages)
            response_text = result.get('response', '')
        else:
            self.clear_message_file()
            response_text = result

        # Extract tool calls and user-friendly responses. / 提取工具调用和用户友好的回复。
        user_friendly_response = response_text
        try:
            json_str = self.extract_json_content(response_text)
            response_json = json.loads(json_str)

            # Extract the tool list. / 提取工具列表。
            tools_list = response_json.get("tools", [])

            # Extract user-friendly response text instead of technical JSON. / 提取用户友好的回复文本，而不是技术性的JSON。
            extracted_response = response_json.get("response", "")
            #self.get_logger().info(f'extracted_response:  {extracted_response}')
            if extracted_response and not extracted_response.startswith('{'):
                # If the response field contains user-friendly text, use it. / 如果response字段包含用户友好的文本，使用它。
                user_friendly_response = extracted_response
            else:
                # Otherwise, generate a friendly response. / 否则生成一个友好的回复。
                if self.regional_setting == "international": 
                    if tools_list:
                        tool_names = [tool.get('name', 'unknown') if isinstance(tool, dict) else str(tool) for tool in tools_list]
                        user_friendly_response = f"Okay, I'll perform these operations for you：{', '.join(tool_names)}"
                    else:
                        user_friendly_response = "I am currently processing your request..."
                else:
                    if tools_list:
                        tool_names = [tool.get('name', 'unknown') if isinstance(tool, dict) else str(tool) for tool in tools_list]
                        user_friendly_response = f"好的，我来为您执行这些操作：{', '.join(tool_names)}"
                    else:
                        user_friendly_response = "我正在处理您的请求..."
            
            self.get_logger().info(f'JSON parsed successfully, extracted {len(tools_list)} tools')
        except Exception as e:
            self.get_logger().warning(f'JSON parsing from response failed: {e}. Using raw response.')
            tools_list = []

        # If the call is from `seewhat`, block recursive `seewhat` calls to prevent loops.
        # 如果调用来自`seewhat`，则阻止递归调用`seewhat`，以防陷入死循环。
        if from_seewhat:
            # block repeated seewhat calls. / 阻止重复的 seewhat 调用。
            filtered_tools = []
            for tool in tools_list:
                if isinstance(tool, dict):
                    tool_name = tool.get("name")
                    if tool_name == "seewhat":
                        self.get_logger().info("Blocking repeated seewhat call to prevent a loop")
                        continue
                elif isinstance(tool, str) and "seewhat" in tool:
                    self.get_logger().info("Blocking repeated seewhat call to prevent a loop")
                    continue
                filtered_tools.append(tool)

            if len(filtered_tools) != len(tools_list):
                self.get_logger().info(f"Filtered, {len(filtered_tools)}/{len(tools_list)} tools remaining")
            tools_list = filtered_tools

        # 统一使用 _provide_feedback 方法处理反馈，无论是否有工具要执行
        # 这样可以确保在语音模式下既播报又打印日志
        if user_friendly_response:
            self._provide_feedback(user_friendly_response)

        self.get_logger().info(f'Preparing to execute tool list: {len(tools_list)} tools total')
        self.execute_tools(tools_list)

    def execute_tools(self, tools_list):
            """
            - Intelligently schedule and execute the tool list.
            - For simple, sequential toolchains, execute directly.
            - For complex tasks requiring planning, call the AI Agent.
            /
            - 智能调度和执行工具列表。
            - 对于简单的、顺序的工具链，直接执行。
            - 对于复杂的、需要规划的任务，调用AI Agent。
            """
            self.get_logger().info(f'Intelligently scheduling tool chain with {len(tools_list)} tools: {tools_list}')
            if not tools_list:
                return

            # Intelligent scheduling logic. / 智能调度逻辑。
            is_complex_task = False
            if len(tools_list) > 1:
                # If the number of tools is greater than 1, we default to it being a complex task that requires planning. / 如果工具数量大于1，我们默认这是一个需要规划的复杂任务。
                is_complex_task = True
            elif tools_list:
                # Check if the only tool is agent_call. / 检查唯一的工具是否是agent_call。
                tool_name = self._extract_tool_name(tools_list[0])
                if tool_name == "agent_call":
                    is_complex_task = True

            if is_complex_task:
                # For complex tasks, we build a task description and hand it over to the AI Agent. / 对于复杂任务，我们构建一个任务描述并交给AI Agent。
                self.get_logger().info(f"Complex task detected - processing with AI Agent")
                
                # Build a more natural language task description. / 构建一个更自然的语言任务描述。
                if self.language == 'zh':
                    task_description = "请完成以下任务："
                else:
                    task_description = "Please complete the following task:"
                    
                tool_details = []
                for tool in tools_list:
                    tool_name = self._extract_tool_name(tool)
                    arguments = self._extract_tool_arguments(tool)
                    if tool_name == "agent_call" and arguments.get("task"):
                        # If it is an agent_call, directly use its task description. / 如果是agent_call，直接使用其任务描述。
                        task_description = str(arguments.get("task", ""))
                        break
                    elif arguments:
                        if self.language == 'zh':
                            tool_details.append(f"使用 {tool_name} 工具，参数为 {arguments}")
                        else:
                            tool_details.append(f"Use {tool_name} tool with arguments {arguments}")
                    else:
                        if self.language == 'zh':
                            tool_details.append(f"使用 {tool_name} 工具")
                        else:
                            tool_details.append(f"Use {tool_name} tool")
                
                if tool_details:
                    if self.language == 'zh':
                        task_description += "；".join(tool_details)
                    else:
                        task_description += "; ".join(tool_details)

                self._execute_agent_task(task_description)

            else:
                # For simple, sequential toolchains, execute directly. / 对于简单的、顺序的工具链，直接执行。
                self.get_logger().info("Simple toolchain detected - executing sequentially")
                outputs = self.tools_manager.execute_tool_chain(tools_list)
                
                if outputs:
                    final_output = outputs[-1]
                    if final_output.success:
                        feedback_data = final_output.data
                        if self.language == 'zh':
                            success_msg = f"工具 {final_output.tool_name} 执行成功。"
                        else:
                            success_msg = f"Tool {final_output.tool_name} executed successfully."
                            
                        if isinstance(feedback_data, dict):
                            feedback_message = feedback_data.get("status_message") or feedback_data.get("description") or success_msg
                        else:
                            feedback_message = str(feedback_data) if feedback_data else success_msg
                        self._provide_feedback(feedback_message)
                    else:
                        if self.language == 'zh':
                            error_message = f"抱歉，执行工具 {final_output.tool_name} 时出错: {final_output.error_message}"
                        else:
                            error_message = f"Sorry, an error occurred while executing tool {final_output.tool_name}: {final_output.error_message}"
                        self._provide_feedback(error_message)
                else:
                    self.get_logger().warning("Toolchain execution completed but produced no output.")

    def _extract_tool_name(self, tool_call):
        """Extract tool name. / 提取工具名称。"""
        if isinstance(tool_call, dict):
            return tool_call.get("name", "")
        elif isinstance(tool_call, str):
            # Handle string format, e.g., "seewhat()" or "agent_call('task')". / 处理字符串格式，如 "seewhat()" 或 "agent_call('task')"。
            if "(" in tool_call:
                return tool_call.split("(")[0]
            return tool_call
        return ""

    def _extract_tool_arguments(self, tool_call):
        """Extract tool arguments. / 提取工具参数。"""
        if isinstance(tool_call, dict):
            return tool_call.get("arguments", {})
        elif isinstance(tool_call, str):
            if "agent_call" in tool_call and "'" in tool_call:
                parts = tool_call.split("'")
                if len(parts) >= 2:
                    return {"task": parts[1]}
            return {}
        return {}

    def _execute_agent_task(self, task_description):
        """ Execute AI Agent task and provide detailed text feedback. / 执行AI Agent任务，并提供详细的文字反馈。"""
        try:
            self.get_logger().info(f"Executing AI Agent task: {task_description}")
            
            # Pass the _provide_feedback function as a callback to the AIAgent.
        # This allows the AIAgent to send real-time updates during execution.
        #/ 将 _provide_feedback 函数作为回调传递给 AIAgent。
        # 这样 AIAgent 就可以在执行过程中，通过这个回调实时发送更新。
            self.ai_agent.feedback_callback = self._provide_feedback
            
            if self.language == 'zh':
                no_task_desc = "未提供任务描述"
                failure_prefix = "❌ AI Agent任务执行失败: "
                unknown_error = "未知错误"
            else:
                no_task_desc = "No task description provided"
                failure_prefix = "❌ AI Agent task execution failed: "
                unknown_error = "Unknown error"
                
            safe_task_description = str(task_description) if task_description is not None else no_task_desc
            agent_result = self.ai_agent.execute_task(safe_task_description)

            # Final success or failure message. / 最终的成功或失败消息。
            if agent_result and agent_result.get("success"):
                if self.language == 'zh':
                    success_msg = f"✅ AI Agent任务完成！\n📊 共执行 {agent_result.get('steps_executed', 0)} 个步骤。"
                else:
                    success_msg = f"✅ AI Agent task completed!\n📊 Executed {agent_result.get('steps_executed', 0)} steps in total."
                self._provide_feedback(success_msg)
            else:
                error_msg = agent_result.get("message", unknown_error) if agent_result else unknown_error
                self._provide_feedback(f"{failure_prefix}{error_msg}")            
            # if self.language == 'zh':
            #     no_task_desc = "未提供任务描述"
            #     success_msg = f"✅ AI Agent任务完成！\n📊 共执行 {agent_result.get('steps_executed', 0)} 个步骤。"
            #     failure_prefix = "❌ AI Agent任务执行失败: "
            #     unknown_error = "未知错误"
            # else:
            #     no_task_desc = "No task description provided"
            #     success_msg = f"✅ AI Agent task completed!\n📊 Executed {agent_result.get('steps_executed', 0)} steps in total."
            #     failure_prefix = "❌ AI Agent task execution failed: "
            #     unknown_error = "Unknown error"
                
            # safe_task_description = str(task_description) if task_description is not None else no_task_desc
            # agent_result = self.ai_agent.execute_task(safe_task_description)

            # # Final success or failure message. / 最终的成功或失败消息。
            # if agent_result and agent_result.get("success"):
            #     self._provide_feedback(success_msg)
            # else:
            #     error_msg = agent_result.get("message", unknown_error)
            #     self._provide_feedback(f"{failure_prefix}{error_msg}")

        except Exception as e:
            self.get_logger().error(f"Critical error during AI Agent execution: {e}")
            if self.language == 'zh':
                error_msg = f"❌ AI Agent执行时出现严重错误: {e}"
            else:
                error_msg = f"❌ Critical error during AI Agent execution: {e}"
            self._provide_feedback(error_msg)
        finally:
            # Clean up the callback to avoid memory leaks. / 清理回调，避免内存泄漏。
            if hasattr(self.ai_agent, 'feedback_callback'):
                self.ai_agent.feedback_callback = None

    def _extract_clean_text_from_mcp_response(self, response_str: str) -> str:
        """Safely extract plain text content from an MCP JSON response. / 从MCP JSON响应中安全地提取纯文本内容。"""
        try:
            if isinstance(response_str, str) and response_str.strip().startswith('{'):
                response_dict = json.loads(response_str)
                content = response_dict.get('result', {}).get('content', [])
                if content and isinstance(content, list):
                    text_item = content[0]
                    if isinstance(text_item, dict) and text_item.get('type') == 'text':
                        return text_item.get('text', response_str)
        except Exception:
            # If parsing fails, return the original string. / 如果解析失败，返回原始字符串。
            pass
        return response_str

    def _clean_text_for_speech(self, text):
        """
        Clean text from Markdown formatting symbols to make it suitable for speech synthesis.
        Simplified version for minimal cleaning.
        清理文本中的Markdown格式符号，使其适合语音播报
        简化版本，只进行最基本的清理
        """
        if not text:
            return text
        
        import re
        
        try:
            # Basic cleaning operations for common Markdown symbols
            # 基本的Markdown符号清理操作
            # Remove heading markers, list markers, and formatting symbols
            # 移除标题标记、列表标记和格式符号
            text = re.sub(r'^#{1,6}\s*', '', text, flags=re.MULTILINE)  # Headings / 标题
            text = re.sub(r'^\s*[-+*]\s+', '', text, flags=re.MULTILINE)  # Unordered lists / 无序列表
            text = re.sub(r'^\s*\d+\.\s+', '', text, flags=re.MULTILINE)  # Ordered lists / 有序列表
            text = re.sub(r'\*\*\*(.*?)\*\*\*', r'\1', text)  # Bold and italic (***text***)
            text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)  # Bold (**text**)
            text = re.sub(r'\*(.*?)\*', r'\1', text)  # Italic (*text*)
            text = re.sub(r'___(.*?)___', r'\1', text)  # Bold and italic (___text___)
            text = re.sub(r'__(.*?)__', r'\1', text)  # Bold (__text__)
            text = re.sub(r'_(.*?)_', r'\1', text)  # Italic (_text_)
            text = re.sub(r'```[\s\S]*?```', '', text)  # Code blocks / 代码块
            text = re.sub(r'`([^`]*)`', r'\1', text)  # Inline code / 行内代码
            text = re.sub(r'$([^\]]*)$$[^\)]*$$', r'\1', text)  # Links / 链接
            text = re.sub(r'!$([^\]]*)$$[^\)]*$$', '', text)  # Images / 图片
            text = re.sub(r'^>\s*', '', text, flags=re.MULTILINE)  # Blockquotes / 引用
            
            # Clean up extra whitespace and newlines
            # 清理多余的空白和换行
            text = re.sub(r'\n\s*\n\s*\n+', '\n\n', text)
            text = re.sub(r' +', ' ', text)
            text = text.strip()
            
            # Remove or replace special characters that may cause TTS issues
            # 移除或替换可能导致TTS问题的特殊字符
            text = re.sub(r'[\u2013\u2014]', '-', text)  # En/em dash
            text = re.sub(r'[\u2018\u2019]', "'", text)  # Single quotes
            text = re.sub(r'[\u201c\u201d]', '"', text)  # Double quotes
            text = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', '', text)  # Control characters
            
            # Limit text length to prevent TTS issues with very long texts
            # 限制文本长度以防止TTS处理超长文本时出现问题
            max_length = 2000
            if len(text) > max_length:
                text = text[:max_length].rsplit(' ', 1)[0] + '...'
            
        except Exception as e:
            self.get_logger().warning(f"Error during text cleaning for speech: {e}. Using original text.")
            return text[:2000] if len(text) > 2000 else text
        
        return text

    def _provide_feedback(self, message):
        """Unified feedback mechanism. / 统一的反馈机制。"""
        feedback_message = str(message)

        self.get_logger().info(f"{feedback_message}")
        # 发布文本消息
        text_msg = String(data=feedback_message)
        self.text_pub.publish(text_msg)

        # 清理文本中的Markdown格式符号，使其适合语音播报
        try:
            cleaned_message = self._clean_text_for_speech(feedback_message)
            
        except Exception as e:
            self.get_logger().error(f"Exception in _clean_text_for_speech: {e}")
            cleaned_message = feedback_message  # Use original message if cleaning fails
        # 合成并播放语音
        self._safe_play_audio(cleaned_message)


    def play_audio(self, file_path, feedback=False):
        """Play audio file synchronously. / 同步方式播放音频函数。"""
        try:
            pygame.mixer.music.load(file_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                if self.stop_event.is_set():
                    pygame.mixer.music.stop()
                    self.stop_event.clear()
                    return
                pygame.time.delay(100)
        except Exception as e:
            self.get_logger().error(f"Error playing audio: {e}")

    def play_audio_async(self, file_path, feedback=False):
        """Play audio asynchronously. / 异步方式播放音频函数。"""
        if self.current_thread and self.current_thread.is_alive():
            self.stop_event.set()
            self.current_thread.join()
            self.stop_event.clear()
        
        def target():
            self.play_audio(file_path, feedback)
        
        self.current_thread = threading.Thread(target=target)
        self.current_thread.daemon = True
        self.current_thread.start()

    def _safe_play_audio(self, text_to_speak: str):
        """
        Synthesizes and plays all non-empty messages only in non-text chat mode.
        /
        仅在非文字聊天模式下，对所有非空消息合成并播放语音。
        """
        if not self.text_chat_mode and text_to_speak:
            try:
                # Synthesize and broadcast all valid incoming feedback messages. / 对所有传入的有效反馈信息都进行语音播报。
                self.model_client.voice_synthesis(text_to_speak, self.tts_out_path)
                self.play_audio_async(self.tts_out_path)
            except Exception as e:
                self.get_logger().error(f"Safe audio playback failed: {e}")

    @staticmethod
    def extract_json_content(raw_content):
        """Extracts a JSON string from raw text content, handling various formats. / 从原始文本内容中提取JSON字符串，处理各种格式。"""
        try:
            # Method 1: Split the code block. / 方法一：分割代码块。
            if '```json' in raw_content:
                # Split the code block and take the middle part. / 分割代码块并取中间部分。
                json_str = raw_content.split('```json')[1].split('```')[0].strip()
            elif '```' in raw_content:
                # Handle code blocks without a specified type. / 处理没有指定类型的代码块。
                json_str = raw_content.split('```')[1].strip()
            else:
                # Method 2: Use a more powerful JSON extraction method. / 方法二：使用更强大的JSON提取方法。
                json_str = LargeModelService._extract_json_from_text(raw_content)

            # Method 3: Fallback simple regular expression (if the above methods fail). / 方法三：备用的简单正则表达式（如果上面的方法失败）。
            if not json_str or not json_str.strip().startswith('{'):
                import re
                match = re.search(r'\{.*\}', raw_content, re.DOTALL)
                if match:
                    json_str = match.group()

            # Check if possible JSON content was found. / 检查是否找到了可能的JSON内容。
            if not json_str or not json_str.strip().startswith('{'):
                # Handle special characters to make the text safe for JSON. / 处理特殊字符，使文本安全用于JSON。
                # Replace all characters that could cause JSON parsing errors. / 替换所有可能导致JSON解析错误的字符。
                safe_content = raw_content.replace('\\', '\\\\')  # Handle backslashes first. / 先处理反斜杠。
                safe_content = safe_content.replace('"', '\\"')    # Handle quotes. / 处理引号。
                safe_content = safe_content.replace('\n', '\\n')   # Handle newlines. / 处理换行。
                safe_content = safe_content.replace('\r', '\\r')   # Handle carriage returns. / 处理回车。
                safe_content = safe_content.replace('\t', '\\t')   # Handle tabs. / 处理制表符。
                safe_content = safe_content.replace('\b', '\\b')   # Handle backspaces. / 处理退格。
                safe_content = safe_content.replace('\f', '\\f')   # Handle form feeds. / 处理换页。

                return '{"response": "' + safe_content + '"}'

            return json_str

        except Exception as e:
            # Also handle special characters. / 同样处理特殊字符。
            if raw_content:
                safe_content = str(raw_content)
                safe_content = safe_content.replace('\\', '\\\\')
                safe_content = safe_content.replace('"', '\\"')
                safe_content = safe_content.replace('\n', '\\n')
                safe_content = safe_content.replace('\r', '\\r')
                safe_content = safe_content.replace('\t', '\\t')
                safe_content = safe_content.replace('\b', '\\b')
                safe_content = safe_content.replace('\f', '\\f')

                return '{"response": "' + safe_content + '"}'
            return '{}'

    @staticmethod
    def _extract_json_from_text(text):
        """Extract a complete JSON object from text, supporting nested structures. / 从文本中提取完整的JSON对象，支持嵌套结构。"""
        # Find the position of the first {. / 找到第一个 { 的位置。
        start_pos = text.find('{')
        if start_pos == -1:
            return ""

        # Starting from the first {, find the matching }. / 从第一个 { 开始，找到匹配的 }。
        brace_count = 0
        in_string = False
        escape_next = False

        for i, char in enumerate(text[start_pos:], start_pos):
            if escape_next:
                escape_next = False
                continue

            if char == '\\':
                escape_next = True
                continue

            if char == '"' and not escape_next:
                in_string = not in_string
                continue

            if not in_string:
                if char == '{':
                    brace_count += 1
                elif char == '}':
                    brace_count -= 1
                    if brace_count == 0:
                        # A complete JSON object was found. / 找到了完整的JSON对象。
                        return text[start_pos:i+1]

        # If a complete JSON object is not found, return an empty string. / 如果没有找到完整的JSON，返回空字符串。
        return ""

    def cleanup_temp_image(self):
        """Clean up temporary image files. / 清理临时图片文件。"""
        try:
            # Clean up temporary image files taken by the camera. / 清理摄像头拍摄的临时图片文件。
            temp_dir = os.path.join(self.pkg_path, "resources_file", "temp_images")
            if os.path.exists(temp_dir):
                import shutil
                shutil.rmtree(temp_dir)
                self.get_logger().info("Temporary image files cleaned up")
        except Exception as e:
            self.get_logger().warning(f"Failed to cleanup temporary image files: {e}")

def main(args=None):
    rclpy.init(args=args)
    model_service = LargeModelService()
    try:
        rclpy.spin(model_service)
    except KeyboardInterrupt:
        model_service.get_logger().info("KeyboardInterrupt received, shutting down...")
    except Exception as e:
        model_service.get_logger().error(f"Unexpected error: {e}")
    finally:
        # Clean up temporary files. / 清理临时文件。
        model_service.cleanup_temp_image()
        model_service.cleanup_message_file()
        model_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()