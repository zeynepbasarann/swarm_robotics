import numpy as np
import gymnasium as gym
from gymnasium import spaces
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, Vector3
from std_msgs.msg import Float32

class MultiRobotEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.min_target_distance = 1.0    #Karşılıklı min. mesafe
        self.target_reach_threshold = 0.8  #Hedefe ulaşması için gerekli min. mesafe
        self.collision_threshold = 0.2   #Çarpışma olması için gerekli min. mesafe

        rclpy.init(args=None)   #ROS2 ye istek
        self.node = rclpy.create_node('multi_robot_env')

        self.robot_names = ['robot1', 'robot2']
        self.cmd_pubs = {}  #Hız
        self.pose_subs = {}  #Pozisyon
        self.orientation_subs = {}  #Yön açısı
        self.front_sensor_subs = {} #Sensör

        self.robot_states = {
            name: {
                'position': np.zeros(3),
                'orientation': 0.0,
                'front_sensors': np.ones(3),
                'prev_position': np.zeros(3),
                'stuck_counter': 0
            } for name in self.robot_names
        }

        for name in self.robot_names:
            self.cmd_pubs[name] = self.node.create_publisher(Twist, f'/{name}/cmd_vel', 10)  #Hız komutunun gönderilmesi
            self.pose_subs[name] = self.node.create_subscription(Point, f'/{name}/position', self._make_position_callback(name), 10)    #Robotun durumunu alma
            self.orientation_subs[name] = self.node.create_subscription(Float32, f'/{name}/orientation', self._make_orientation_callback(name), 10)
            self.front_sensor_subs[name] = self.node.create_subscription(Vector3, f'/{name}/front_sensors', self._make_front_sensor_callback(name), 10)

        self.previous_targets = []  #Eski hedeflerin listesi
        self.current_target = np.zeros(2)   ##Şimdiki hedef
        self.robot2_has_target = False   #Robot2 ye hedef verilip verilmemesi 

        self.action_space = spaces.MultiDiscrete([6, 6])  #PPO için eylem
        self.observation_space = spaces.Box(low=-2.0, high=2.0, shape=(14,), dtype=np.float32)  #Gözlem boyutu 14

        self.success_count = 0  #Görev başarısı ve ortalama ödül
        self.failure_count = 0
        self.reward_log = []

        self.current_target = self._sample_target_position()  #Başlangıçta belirlenen hedef

    def _sample_target_position(self):
        while True:
            new_target = np.random.uniform(-1.5, 1.5, size=2)   #Rastgele hedef üretilmesi
            too_close = False

            if 'robot1' in self.robot_states:  #Üretilen hedef Robot1'e çok yakınsa tekrar üret
                if np.linalg.norm(new_target - self.robot_states['robot1']['position'][:2]) < self.min_target_distance:
                    continue

            for old in self.previous_targets:  #Eski hedefe çok yakınsa tekrar üret
                if np.linalg.norm(new_target - old) < self.min_target_distance:
                    too_close = True
                    break

            if np.linalg.norm(new_target - self.current_target) < self.min_target_distance:
                continue

            if not too_close:  #Üretileni mevcut hedef yap
                return new_target

    def _make_position_callback(self, name):  #ROS2 den gelen mesajlarla robotun konumunu güncelleme
        def callback(msg):
            pos = np.array([msg.x, msg.y, msg.z])
            self.robot_states[name]['position'] = pos
        return callback

    def _make_orientation_callback(self, name):  #ROS2 den gelen mesajlarla robotun yönünü güncelleme
        def callback(msg):
            self.robot_states[name]['orientation'] = msg.data
        return callback

    def _make_front_sensor_callback(self, name):  #ROS2 den gelen mesajlarla sensör mesafelerini güncelleme
        def callback(msg):
            self.robot_states[name]['front_sensors'] = np.array([msg.x, msg.y, msg.z])
        return callback

    def _navigate_to_target(self, name, target): #Robotun hedefe gitmesini sağlar
        twist = Twist()
        state = self.robot_states[name]
        dx, dy = target[0] - state['position'][0], target[1] - state['position'][1]  #Robotla hedef arasındaki açıyı hesaplar
        target_angle = np.arctan2(dy, dx)
        current_angle = state['orientation']
        angle_diff = np.arctan2(np.sin(target_angle - current_angle), np.cos(target_angle - current_angle)) #Pozitifse sağda negatifse solda

        min_sensor = np.min(state['front_sensors'])
        left, mid, right = state['front_sensors']

        if min_sensor < self.collision_threshold:  #Sensörlerle çarpışma kontrolü
            twist.linear.x = -0.4
            twist.angular.z = 2.0 if left < right else -2.0
        elif abs(angle_diff) > 0.3:
            twist.angular.z = 2.0 if angle_diff > 0 else -2.0
        else:
            twist.linear.x = 1.0

        self.cmd_pubs[name].publish(twist)  #Yapılacak hareketi gönderme

    def _navigate(self, name, action):  #DRL ajanı ile actiona göre robotu yönlendirme
        twist = Twist()
        state = self.robot_states[name]
        front = state['front_sensors']
        min_front = np.min(front)
        left, mid, right = front

        if min_front < self.collision_threshold:
            twist.linear.x = -0.4
            twist.angular.z = 2.0 if left < right else -2.0
            state['stuck_counter'] += 1
        else:
            state['stuck_counter'] = 0
            if action == 0:
                twist.linear.x = 1.0
            elif action == 1:
                twist.linear.x = -1.0
            elif action == 2:
                twist.angular.z = 2.0
            elif action == 3:
                twist.angular.z = -2.0
            elif action == 4:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            elif action == 5:
                twist.linear.x = np.random.uniform(0.5, 1.0)
                twist.angular.z = np.random.uniform(-2.0, 2.0)

        movement = np.linalg.norm(state['position'][:2] - state['prev_position'][:2])
        if movement < 0.01:  #Hareket ediyor mu?
            state['stuck_counter'] += 1
        else:
            state['stuck_counter'] = 0

        if state['stuck_counter'] >= 10: #Uzun süre sıkıştı mı?
            twist.linear.x = -0.5
            twist.angular.z = np.random.choice([-2.0, 2.0])
            state['stuck_counter'] = 0

        state['prev_position'] = state['position'].copy()
        self.cmd_pubs[name].publish(twist) #Hareket ettirme

    def _get_obs(self):  #Mevcut durumu öğrenme alg a alır.Model bu bilgilerle hareket eder
        obs = []
        for name in self.robot_names:
            state = self.robot_states[name]
            obs.extend(state['position'][:2])
            obs.append(state['orientation'])
            obs.extend(state['front_sensors'])
        obs.extend(self.current_target)
        return np.array(obs, dtype=np.float32)

    def step(self, actions):
        for _ in range(3):  #güncel sensör ve posizyon bilgilerini alır.
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self._navigate_to_target('robot1', self.current_target) # Robot1 görevlendirilir

        if self.robot2_has_target:  #Robot2 görevlendirilme
            self._navigate_to_target('robot2', self.previous_targets[0])
        else:
            twist = Twist()
            self.cmd_pubs['robot2'].publish(twist)
        #Ödül Hesaplama
        r1_state = self.robot_states['robot1']
        r1_pos = r1_state['position'][:2]
        r1_prev_pos = r1_state['prev_position'][:2]
        r1_sensors = r1_state['front_sensors']

        dist = np.linalg.norm(r1_pos - self.current_target)
        prev_dist = np.linalg.norm(r1_prev_pos - self.current_target)

        reward = -0.01  # küçük zaman cezası

        if dist < self.target_reach_threshold:
            print("🎯 Robot1 hedefe ulaştı. Hedef Robot2'ye aktarıldı.")
            self.previous_targets = [self.current_target.copy()]
            self.current_target = self._sample_target_position()
            self.robot2_has_target = True
            self.success_count += 1
            reward += 5.0

        elif dist < prev_dist:
            reward += 0.5  # hedefe yaklaşma ödülü
        else:
            reward -= 0.5  # uzaklaşma cezası

        if np.min(r1_sensors) < self.collision_threshold:
            reward -= 1.0  # çarpışma cezası
        
        if r1_state['stuck_counter'] > 5:
            reward -= 2.0  # çok uzun süre aynı yerdeyse

        #Çıktı 
        self.reward_log.append(reward)

        obs = self._get_obs()
        done = False
        truncated = False
        info = {}

        return obs, reward, done, truncated, info


    def reset(self, *, seed=None, options=None):  #Her episode den sonra ortamı sıfırlamak
        super().reset(seed=seed)
        self.current_target = self._sample_target_position() #Yeni konum

        for _ in range(20): #Güncel bilgiyi alma
            rclpy.spin_once(self.node, timeout_sec=0.1)

        while np.linalg.norm(self.current_target - self.robot_states['robot1']['position'][:2]) < self.min_target_distance:
            self.current_target = self._sample_target_position()

        print(f"🎯 Başlangıç hedefi robot1 için: {self.current_target}")
        #Temizlik
        self.previous_targets = []
        self.robot2_has_target = False
        self.reward_log = []
        #Robotları durdur
        for name in self.robot_names:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            for _ in range(3):
                self.cmd_pubs[name].publish(twist)
                rclpy.spin_once(self.node, timeout_sec=0.1)
        return self._get_obs(), {}
    #Ortam Kapatma
    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def report_performance(self):
        total = self.success_count + self.failure_count
        success_rate = (self.success_count / total) * 100 if total > 0 else 0
        print("\n📊 PERFORMANS RAPORU")
        print(f"🔹 Başarılı görev sayısı   : {self.success_count}")
        print(f"🔹 Başarı oranı           : {success_rate:.2f}%")
        print(f"🔹 Ortalama ödül          : {np.mean(self.reward_log):.2f} (son görev)")
        print(f"🔹 Başarısız görev sayısı : {self.failure_count}")
