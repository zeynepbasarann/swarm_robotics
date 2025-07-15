from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from multi_robot_env import MultiRobotEnv
import time

def test():
    env = MultiRobotEnv()
    vec_env = DummyVecEnv([lambda: env])

    model = PPO.load("multi_robot_model_200000", env=vec_env)

    obs = vec_env.reset()
    for _ in range(1000):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = vec_env.step(action)

        time.sleep(0.1)

    env.report_performance()
    env.close()

if __name__ == "__main__":
    test()
