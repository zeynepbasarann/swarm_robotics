from stable_baselines3 import PPO
from multi_robot_env import MultiRobotEnv
from stable_baselines3.common.vec_env import DummyVecEnv


def main():
    env = MultiRobotEnv()
    vec_env = DummyVecEnv([lambda: env])
    model = PPO("MlpPolicy", vec_env, verbose=1, tensorboard_log="./ppo_logs")

    for i in range(20):  # Toplam 500.000 adım
        model.learn(total_timesteps=10000, reset_num_timesteps=False)
        model.save(f"multi_robot_model_{(i+1)*10000}")

    env.close()


if __name__ == "__main__":
    main()
