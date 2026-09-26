# 本机依赖与安装副本补查

2026-09-15；只读取文件，不导入SDK、不打开设备。主扫描不计虚拟环境，以下是针对控制链边界的单独补查；这些路径依赖本机环境，迁移后必须复核。

| 文件 | SHA256 |
|---|---|
| `dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/__init__.py` | `d0ef7964adffb75e1a38050efa80bba30a8b818ba3c08389373b07b2696c0954` |
| `dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/interface/piper_interface.py` | `ee63f0fa68286f7d0970ac9b53828c407f135ee68624064404fc419b0acede92` |
| `dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/hardware_port/can_encapsulation_v0_4_0.py` | `9a75a19d92073f2842b02cdc53475306345013160840401b9f58e297b273e4d3` |
| `dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/protocol/protocol_v2/piper_protocol_v2.py` | `e244ce3ef8742e468df4d0e37b1c845cffb4e7e3cf65a9405ccbf5a3edd5c867` |
| `dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/motorbridge/core.py` | `9f6e7998f1e2ac9d42f6905a000ddd8f596adcc66fb5ad6ddbd0ad366a467d24` |

## Piper真实发送路径

PiperRosNode使用piper_sdk.C_PiperInterface；安装SDK JointCtrl拆成12/34/56关节对→EncodeMessage→SendCanMessage→can.Message→bus.send。CAN ID为0x155/0x156/0x157（也出现在Leader模式反馈，方向和角色必须区分）；标准CAN默认8字节，expected_bitrate=1000000。该值是SDK要求，不是已测can0实际bitrate。Piper控制器内部电机路由仍UNKNOWN。

证据：[`dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/interface/piper_interface.py:2716`](../../dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/interface/piper_interface.py#L2716)

```python
    def JointCtrl(self, 
                  joint_1: int, 
                  joint_2: int,
                  joint_3: int,
                  joint_4: int,
                  joint_5: int,
                  joint_6: int):
        '''
```

证据：[`dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/interface/piper_interface.py:2779`](../../dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/interface/piper_interface.py#L2779)

```python
    def __JointCtrl_12(self, joint_1: int, joint_2: int):
        '''
        机械臂1,2关节控制
        
        私有函数
        
        Args:
            joint_1 (int): 关节1角度,单位0.001度
```

证据：[`dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/hardware_port/can_encapsulation_v0_4_0.py:187`](../../dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/hardware_port/can_encapsulation_v0_4_0.py#L187)

```python
    def SendCanMessage(self, arbitration_id, data, dlc=8, is_extended_id=False):
        '''can transmit

        Args:
            arbitration_id (_type_): _description_
            data (_type_): _description_ Defaults to 8.
            is_extended_id_ (bool, optional): _description_. Defaults to False.
        '''
```

证据：[`dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/hardware_port/can_encapsulation_v0_4_0.py:85`](../../dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/hardware_port/can_encapsulation_v0_4_0.py#L85)

```python
                 expected_bitrate:int=1000000,
                 judge_flag:bool=True, 
                 auto_init:bool=True,
                 callback_function: Callable = None) -> None:
        self.channel_name = channel_name
        self.bustype = bustype
        self.expected_bitrate = expected_bitrate
        self.rx_message:Optional[Message] = Message()   #创建消息接收类
```

证据：[`dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/protocol/protocol_v2/piper_protocol_v2.py:351`](../../dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/piper_sdk/protocol/protocol_v2/piper_protocol_v2.py#L351)

```python
        elif(msg_type_ == ArmMsgType.PiperMsgJointCtrl_12):
            tx_can_frame.data = self.ConvertToList_32bit(msg.arm_joint_ctrl.joint_1) + \
                                self.ConvertToList_32bit(msg.arm_joint_ctrl.joint_2)
        elif(msg_type_ == ArmMsgType.PiperMsgJointCtrl_34):
            tx_can_frame.data = self.ConvertToList_32bit(msg.arm_joint_ctrl.joint_3) + \
                                self.ConvertToList_32bit(msg.arm_joint_ctrl.joint_4)
        elif(msg_type_ == ArmMsgType.PiperMsgJointCtrl_56):
            tx_can_frame.data = self.ConvertToList_32bit(msg.arm_joint_ctrl.joint_5) + \
```

证据：[`dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/motorbridge/core.py:39`](../../dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/motorbridge/core.py#L39)

```python
    def from_dm_serial(cls, serial_port: str = "/dev/ttyACM0", baud: int = 921600) -> "Controller":
        self = cls.__new__(cls)
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_dm_serial(serial_port.encode(), int(baud))
        if not self._ptr:
            raise CallError(f"new_dm_serial failed: {_err_text()}")
        return self

```

证据：[`dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/motorbridge/core.py:23`](../../dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/motorbridge/core.py#L23)

```python
    def __init__(self, channel: str = "can0") -> None:
        self._abi = get_abi()
        self._ptr = self._abi.lib.motor_controller_new_socketcan(channel.encode())
        if not self._ptr:
            raise CallError(f"new_socketcan failed: {_err_text()}")

    @classmethod
    def from_socketcanfd(cls, channel: str = "can0") -> "Controller":
```

证据：[`dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/motorbridge/core.py:179`](../../dual_arm_ws/.hardware-venv/lib/python3.12/site-packages/motorbridge/core.py#L179)

```python
    def send_pos_vel(self, pos: float, vlim: float) -> None:
        _ok(self._abi.lib.motor_handle_send_pos_vel(self._require_open(), pos, vlim), "send_pos_vel")

    def robstride_send_pos_vel_pp(self, pos: float, vel_max: float, acc_set: float) -> None:
        _ok(
            self._abi.lib.motor_handle_robstride_send_pos_vel_pp(self._require_open(), pos, vel_max, acc_set),
            "robstride_send_pos_vel_pp",
        )
```

## MotorBridge native边界

Controller(channel)调用motor_controller_new_socketcan；from_dm_serial调用motor_controller_new_dm_serial(port,baud)。因此reBot SDK _make_controller对by-id字符串的分支落错不是仅凭函数名称猜测。send_pos_vel→motor_handle_send_pos_vel；send_mit→native ABI；后续libmotor_abi.so源码/串口桥固件及线上的CAN封装未在本仓库确认。库也提供from_socketcanfd/from_dm_device，但当前reBot DM/RS默认路径没有使用，不能据此宣称生产CAN-FD已接入。

## 源码与安装版本

逐字比较dual_arm_ws/src与dual_arm_ws/install对应Python模块：rebotarm_pulse、piperh_control、rebotarmcontroller全部相同（2026-09-15读取时）。这只验证文件一致，不能证明运行进程使用该版本或其他包的安装版本一致。独立start_pulse_pressure.sh显式把root/src/rebotarm_pulse放到PYTHONPATH，与双arm overlay入口仍有来源差别。
