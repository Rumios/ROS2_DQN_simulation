import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk
import threading

class UIControllerNode(Node):
    def __init__(self):
        super().__init__('ui_controller_node')
        
        # /cmd_vel 토픽으로 Twist 메시지 발송 Publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 0.1초마다 키보드 상태 확인 & 명령 내리는 타이머
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.pressed_keys = {
            'Up' : False,
            'Down' : False,
            'Left' : False,
            'Right' : False, 
            }
        
        self.get_logger().info("UI 컨트롤러 시작")
        
    def timer_callback(self):
        msg = Twist()
        
        # pressed -> 1, else 0
        linear_vel = 0.0
        angular_vel = 0.0
        
        if self.pressed_keys['Up']: linear_vel += 2.0 # 직진
        if self.pressed_keys['Down']: linear_vel -= 2.0 # 후진
        if self.pressed_keys['Left']: angular_vel += 2.0 # 좌회전
        if self.pressed_keys['Right']: angular_vel -= 2.0 # 우회전
        
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        
        self.publisher_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    node = UIControllerNode()
    
    root = tk.Tk()
    root.title("DQN Controller UI")
    root.geometry("400x350")
    root.configure(bg="#1E1E1E") # 다크 테마 배경
    
    # 상단 안내 텍스트
    title_label = tk.Label(root, text="ROBOT CONTROLLER", font=("Helvetica", 18, "bold"), bg="#1E1E1E", fg="#FFFFFF")
    title_label.pack(pady=(20, 10))
    sub_label = tk.Label(root, text="↑, ↓, ←, →", font=("Helvetica", 11), bg="#1E1E1E", fg="#AAAAAA")
    sub_label.pack(pady=(0, 20))
    
    # 방향키를 담을 프레임
    key_frame = tk.Frame(root, bg="#1E1E1E")
    key_frame.pack()
    
    # 버튼 기본 스타일 & 눌렸을 때 스타일
    default_style = {"font": ("Helvetica", 24), "width": 4, "height": 2, "bg": "#333333", "fg": "#FFFFFF", "relief": "ridge", "bd": 2}
    active_bg = "#4CAF50" # 눌렸을 때 들어오는 초록색 불빛
    active_fg = "#FFFFFF"
    
    # 십자(Inverted-T) 모양으로 라벨 배치
    btn_up = tk.Label(key_frame, text="▲", **default_style)
    btn_up.grid(row=0, column=1, padx=5, pady=5)
    
    btn_left = tk.Label(key_frame, text="◀", **default_style)
    btn_left.grid(row=1, column=0, padx=5, pady=5)
    
    btn_down = tk.Label(key_frame, text="▼", **default_style)
    btn_down.grid(row=1, column=1, padx=5, pady=5)
    
    btn_right = tk.Label(key_frame, text="▶", **default_style)
    btn_right.grid(row=1, column=2, padx=5, pady=5)
    
    # 이벤트 발생 시 UI 색상을 바꾸기 위해 딕셔너리로 묶음
    ui_elements = {
        'Up': btn_up,
        'Down': btn_down,
        'Left': btn_left,
        'Right': btn_right
    }
    
    def on_key_press(event):
        key = event.keysym # 'Up', 'Down', 'Left', 'Right'
        if key in node.pressed_keys and not node.pressed_keys[key]:
            node.pressed_keys[key] = True
            ui_elements[key].configure(bg=active_bg, fg=active_fg) # 불 켜기
            
    # 키보드에서 손을 뗐을 때 (Release)
    def on_key_release(event):
        key = event.keysym
        if key in node.pressed_keys:
            node.pressed_keys[key] = False
            ui_elements[key].configure(bg=default_style["bg"], fg=default_style["fg"]) # 불 끄기

    root.bind('<KeyPress>', on_key_press)
    root.bind('<KeyRelease>', on_key_release)
    
    # --- ROS 2 통신 스레드 ---
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    