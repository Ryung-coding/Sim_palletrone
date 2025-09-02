import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, VecNormalize
from stable_baselines3.common.env_checker import check_env

from arm_reach_env import ArmReachEnv
from arm_reach_env import LiveViewerCallback

XML = os.path.join(os.path.dirname(__file__), "xml", "scene.xml")

def make_env():
    def _thunk():
        return ArmReachEnv(XML, render_mode=None)
    return _thunk

if __name__ == "__main__":
    # Single env sanity check
    env = ArmReachEnv(XML, render_mode=None)
    check_env(env, warn=True)  # optional, helps catch interface errors
    env.close()

    # Vectorized training envs
    n_envs = 8
    vec_env = SubprocVecEnv([make_env() for _ in range(n_envs)])
    vec_env = VecNormalize(vec_env, norm_obs=True, norm_reward=True, clip_obs=10.0)

    # Live viewer
    eval_env_live = ArmReachEnv(XML, render_mode="human")

    policy_kwargs = dict(net_arch=[256, 256])
    model = PPO(
        "MlpPolicy",
        vec_env,
        n_steps=1024,                # per env rollout length
        batch_size=1024,             # >= n_steps * n_envs / n_minibatch
        n_epochs=10,
        gamma=0.995,
        gae_lambda=0.95,
        learning_rate=3e-4,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        policy_kwargs=policy_kwargs,
        tensorboard_log="./tb",
        device="auto",
        verbose=1,
    )

    # Show the viewer for 1 episode every 50k steps
    viewer_cb = LiveViewerCallback(eval_env_live, eval_freq=50_000, n_eval_episodes=1, viewer_fps=30)

    model.learn(total_timesteps=10_000_000)
    model.save("ppo_arm_reach")
    vec_env.save("vecnorm.pkl")  # to reuse normalization stats later
    vec_env.close()

    eval_env_live.close()
