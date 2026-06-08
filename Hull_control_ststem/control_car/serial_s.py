#!/usr/bin/env python3
"""
交互式串口收发工具
支持即时发送和接收，可发送文本或十六进制
"""

import serial
import serial.tools.list_ports
import threading
import time
import sys

# ==================== 串口配置 ====================
PORT = "/dev/ttyUSB1"      # 串口名 (Linux: /dev/ttyUSB1, Windows: COM3)
BAUDRATE = 115200          # 波特率
TIMEOUT = 1                # 超时时间（秒）

# ==================== 全局变量 ====================
serial_port = None
receive_thread = None
running = True


def list_available_ports():
    """列出所有可用串口"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用串口")
        return []
    print("\n可用串口：")
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p.device} - {p.description}")
    print()
    return [p.device for p in ports]


def open_serial(port: str, baudrate: int) -> bool:
    """打开串口"""
    global serial_port
    try:
        serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT
        )
        print(f"✓ 串口已打开: {port} @ {baudrate} bps\n")
        return True
    except serial.SerialException as e:
        print(f"✗ 打开串口失败: {e}")
        return False


def close_serial():
    """关闭串口"""
    global running, serial_port
    running = False
    if serial_port and serial_port.is_open:
        serial_port.close()
        print("串口已关闭")


def receive_data():
    """接收线程：持续接收数据"""
    global running, serial_port
    buffer = bytearray()
    
    while running and serial_port and serial_port.is_open:
        try:
            if serial_port.in_waiting > 0:
                data = serial_port.read(serial_port.in_waiting)
                
                # 显示接收到的原始字节
                print(f"\n[RX] {data.hex()} | ", end="")
                
                # 尝试显示可打印字符
                try:
                    text = data.decode('utf-8').strip()
                    printable = ''.join(c if c.isprintable() else '.' for c in text)
                    print(f'"{printable}"')
                except:
                    print(f"二进制数据 ({len(data)} 字节)")
                
                print("> ", end="", flush=True)
            
            time.sleep(0.01)
        except Exception as e:
            if running:
                print(f"\n接收错误: {e}")
            break


def send_text(text: str):
    """发送文本"""
    global serial_port
    if not serial_port or not serial_port.is_open:
        print("✗ 串口未打开")
        return
    
    try:
        data = (text + "\r\n").encode('utf-8')
        serial_port.write(data)
        serial_port.flush()
        print(f"[TX] {text}")
    except Exception as e:
        print(f"✗ 发送失败: {e}")


def send_hex(hex_str: str):
    """发送十六进制数据"""
    global serial_port
    if not serial_port or not serial_port.is_open:
        print("✗ 串口未打开")
        return
    
    try:
        # 支持两种格式: "FF 01 02" 或 "FF0102"
        hex_str = hex_str.replace(" ", "").replace("-", "")
        data = bytes.fromhex(hex_str)
        serial_port.write(data)
        serial_port.flush()
        print(f"[TX] {data.hex()}")
    except ValueError:
        print("✗ 无效的十六进制格式")
    except Exception as e:
        print(f"✗ 发送失败: {e}")


def show_help():
    """显示帮助"""
    print("""
╔══════════════════════════════════════════════════╗
║              串口收发 - 命令说明                  ║
╠══════════════════════════════════════════════════╣
║  直接输入文本按回车发送                          ║
║                                                  ║
║  命令:                                           ║
║    :h            - 显示帮助                      ║
║    :hex <data>   - 发送十六进制，如 :hex FF 01 02║
║    :list         - 列出可用串口                   ║
║    :open <port>  - 打开串口                      ║
║    :close        - 关闭串口                       ║
║    :clear        - 清屏                          ║
║    :q / :quit   - 退出程序                       ║
╚══════════════════════════════════════════════════╝
""")


def main():
    """主循环"""
    global running, serial_port, receive_thread
    
    print("=" * 50)
    print("  交互式串口收发工具")
    print("=" * 50)
    
    # 列出可用串口
    available_ports = list_available_ports()
    
    # 自动尝试打开默认串口
    if available_ports:
        if not open_serial(PORT, BAUDRATE):
            # 尝试第一个可用串口
            open_serial(available_ports[0], BAUDRATE)
    
    show_help()
    
    # 启动接收线程
    if serial_port and serial_port.is_open:
        running = True
        receive_thread = threading.Thread(target=receive_data, daemon=True)
        receive_thread.start()
    
    # 主循环：等待用户输入
    try:
        while True:
            try:
                user_input = input("> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n正在退出...")
                break
            
            if not user_input:
                continue
            
            # 命令解析
            if user_input.startswith(":"):
                cmd = user_input[1:].split()[0].lower()
                args = user_input[1:].split()[1:] if len(user_input) > 1 else []
                
                if cmd in ["q", "quit", "exit"]:
                    break
                elif cmd in ["h", "help"]:
                    show_help()
                elif cmd in ["list", "ls"]:
                    list_available_ports()
                elif cmd == "clear":
                    print("\033[2J\033[H")
                elif cmd == "close":
                    close_serial()
                elif cmd == "open":
                    if args:
                        port = args[0]
                        baud = int(args[1]) if len(args) > 1 else BAUDRATE
                        if open_serial(port, baud):
                            running = True
                            receive_thread = threading.Thread(target=receive_data, daemon=True)
                            receive_thread.start()
                    else:
                        print("用法: :open <串口> [波特率]")
                elif cmd == "hex":
                    if args:
                        send_hex(" ".join(args))
                    else:
                        print("用法: :hex FF 01 02")
                else:
                    print(f"✗ 未知命令: {cmd}")
            
            # 普通文本输入 - 直接发送
            else:
                send_text(user_input)
    
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C")
    finally:
        close_serial()
        print("程序已退出")


if __name__ == "__main__":
    main()