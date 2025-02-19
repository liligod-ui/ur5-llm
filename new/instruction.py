# 假设已有的机械臂指令函数
def connect():
    print("机械臂连接成功。")
def disconnect():
    print("机械臂断开连接，资源已释放。")
def move_to(x, y, z):
    print(f"移动至坐标 ({x}, {y}, {z})")
def grab():
    print("执行抓取动作。")
def release():
    print("执行释放动作。")
class RobotArmController:
    def __init__(self):
        self.instructions = []  # 存储指令队列
    def add_instruction(self, func, *args):
        """ 动态添加指令 """
        self.instructions.append((func, args))
    def execute(self):
        """ 执行所有指令并确保资源释放 """
        connect()
        try:
            for func, args in self.instructions:
                try:
                    func(*args)  # 执行指令
                except Exception as e:
                    print(f"执行 {func.__name__} 时出错: {e}")
                    break  # 可选：出错时终止后续指令
        finally:
            disconnect()  # 确保无论如何都释放资源
# 使用示例
if __name__ == "__main__":
    # 创建控制器实例
    controller = RobotArmController()

    # 动态添加指令（可从文件/网络动态读取）
    controller.add_instruction(move_to, 10, 20, 30)
    controller.add_instruction(grab)
    controller.add_instruction(move_to, 0, 0, 0)
    controller.add_instruction(release)

    # 执行指令队列
    controller.execute()