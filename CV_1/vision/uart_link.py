# UART4 封装：MaixCAM2 -> STM32G473
# 接线：A21(UART4_TX) -> PC5(USART1_RX)
#       A22(UART4_RX) -> PC4(USART1_TX)
#       GND 共地
# 参数：115200, 8N1, 设备节点 /dev/ttyS4

from maix import uart, pinmap

UART_PORT = "/dev/ttyS4"
UART_BAUD = 115200


class UartLink:
    def __init__(self):
        # A21/A22 默认不是 UART4，必须先用 pinmap 复用
        pinmap.set_pin_function("A21", "UART4_TX")
        pinmap.set_pin_function("A22", "UART4_RX")
        self.dev = uart.UART(UART_PORT, UART_BAUD)

    def send(self, cx, cy, area):
        """发送一行文本协议：cx,cy,area\n"""
        self.dev.write(f"{cx},{cy},{area}\n".encode())
