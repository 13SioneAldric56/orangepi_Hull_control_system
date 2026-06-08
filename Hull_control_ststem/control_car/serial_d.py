#!/usr/bin/env python3
"""
串口回环测试代码
用于测试串口通信是否正常

使用说明：
1. 硬件回环：将串口的 TX 和 RX 引脚用杜邦线短接
2. 软件回环：串口芯片自带的回环模式（部分串口模块支持）

用法：
    python serial_loopback_test.py /dev/ttyS0
    python serial_loopback_test.py /dev/ttyS0 -b 115200
"""

import serial
import serial.tools.list_ports
import argparse
import time
import sys


def list_available_ports():
    """列出所有可用的串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用的串口")
        return []
    
    print("可用的串口：")
    for i, port in enumerate(ports, 1):
        print(f"  {i}. {port.device} - {port.description}")
    return [p.device for p in ports]


def serial_loopback_test(port: str, baudrate: int = 115200, 
                        test_data: str = "Hello Serial Loopback Test!",
                        timeout: float = 1.0,
                        loop_mode: bool = False) -> bool:
    """
    串口回环测试
    
    Args:
        port: 串口号，如 '/dev/ttyS0' 或 '/dev/ttyUSB1'
        baudrate: 波特率，默认 115200
        test_data: 测试数据字符串
        timeout: 读取超时时间（秒）
        loop_mode: 是否启用软件回环模式（仅部分串口支持）
    
    Returns:
        测试是否成功
    """
    try:
        # 打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        
        # 启用软件回环模式（如果支持）
        if loop_mode:
            try:
                ser.loopback = True
                print(f"[模式] 已启用软件回环模式")
            except AttributeError:
                print(f"[警告] 当前串口不支持软件回环模式，将使用硬件回环")
        
        print(f"\n{'='*50}")
        print(f"串口回环测试")
        print(f"{'='*50}")
        print(f"  串口: {port}")
        print(f"  波特率: {baudrate}")
        print(f"  测试数据: {test_data}")
        print(f"{'='*50}\n")
        
        # 清空缓冲区
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # 发送测试数据
        data_to_send = test_data.encode('utf-8')
        bytes_sent = ser.write(data_to_send)
        ser.flush()
        print(f"[发送] {bytes_sent} 字节: {test_data}")
        
        # 等待数据回环回来
        time.sleep(0.1)
        
        # 读取回环数据
        data_received = ser.read(len(data_to_send))
        
        # 关闭串口
        ser.close()
        
        # 验证结果
        if data_received == data_to_send:
            print(f"[接收] {len(data_received)} 字节: {data_received.decode('utf-8', errors='replace')}")
            print(f"\n✓ 回环测试成功！发送和接收的数据一致")
            return True
        else:
            print(f"[接收] {len(data_received)} 字节: {data_received.hex() if data_received else '(无数据)'}")
            print(f"\n✗ 回环测试失败！数据不匹配")
            print(f"  期望: {data_to_send.hex()}")
            print(f"  实际: {data_received.hex() if data_received else '(无)'}")
            return False
            
    except serial.SerialException as e:
        print(f"\n✗ 串口错误: {e}")
        return False
    except Exception as e:
        print(f"\n✗ 未知错误: {e}")
        return False


def run_comprehensive_test(port: str, baudrate: int = 115200):
    """运行综合测试：多种数据格式"""
    test_cases = [
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ",
        "0123456789",
        "!@#$%^&*()_+-=[]{}|;':\",./<>?",
        "中文测试数据 Chinese Test",
        bytes([0x00, 0x01, 0x02, 0xFF, 0xFE, 0xFD]).decode('latin-1'),
    ]
    
    print(f"\n{'='*50}")
    print(f"综合回环测试 (共 {len(test_cases)} 项)")
    print(f"{'='*50}\n")
    
    passed = 0
    failed = 0
    
    for i, test_data in enumerate(test_cases, 1):
        print(f"[{i}/{len(test_cases)}] 测试: {repr(test_data)}")
        if serial_loopback_test(port, baudrate, test_data, timeout=0.5):
            passed += 1
        else:
            failed += 1
        print()
        time.sleep(0.3)
    
    print(f"{'='*50}")
    print(f"测试结果: {passed}/{len(test_cases)} 通过, {failed}/{len(test_cases)} 失败")
    print(f"{'='*50}")
    
    return failed == 0


def main():
    parser = argparse.ArgumentParser(
        description='串口回环测试工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  列出可用串口:
    python serial_loopback_test.py --list
  
  简单测试:
    python serial_loopback_test.py /dev/ttyS0
  
  指定波特率:
    python serial_loopback_test.py /dev/ttyS0 -b 9600
  
  综合测试:
    python serial_loopback_test.py /dev/ttyS0 --full-test
  
  使用软件回环:
    python serial_loopback_test.py /dev/ttyS0 --loopback
        """
    )
    
    parser.add_argument('port', nargs='?', help='串口号 (如 /dev/ttyS0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, 
                       help='波特率 (默认: 115200)')
    parser.add_argument('-l', '--list', action='store_true',
                       help='列出所有可用的串口')
    parser.add_argument('--full-test', action='store_true',
                       help='运行综合测试')
    parser.add_argument('--loopback', action='store_true',
                       help='启用软件回环模式 (如果支持)')
    parser.add_argument('-t', '--test-data', type=str, default='Hello Serial!',
                       help='自定义测试数据')
    
    args = parser.parse_args()
    
    # 列出串口
    if args.list:
        list_available_ports()
        return
    
    # 如果没有指定串口，列出可用串口
    if not args.port:
        ports = list_available_ports()
        if ports:
            print(f"\n请指定串口号，或使用 --list 查看详情")
        else:
            print("未找到可用串口，请检查连接")
        sys.exit(1)
    
    # 运行测试
    if args.full_test:
        success = run_comprehensive_test(args.port, args.baudrate)
    else:
        success = serial_loopback_test(
            args.port, 
            args.baudrate, 
            args.test_data,
            loop_mode=args.loopback
        )
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()