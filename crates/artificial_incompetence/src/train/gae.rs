pub fn compute_gae(
    rewards: impl Iterator<Item = f64> + DoubleEndedIterator + ExactSizeIterator,
    values: impl Iterator<Item = f64> + DoubleEndedIterator + ExactSizeIterator,
    last_value: f64,
    gamma: f64,
    gae_lambda: f64,
) -> (Vec<f64>, Vec<f64>) {
    let mut advantages = vec![0.0; rewards.len()];
    let mut returns = vec![0.0; rewards.len()];

    let mut gae = 0.0;
    let mut next_value = last_value;

    for ((adv, ret), (reward, value)) in advantages
        .iter_mut()
        .rev()
        .zip(returns.iter_mut().rev())
        .zip(rewards.rev().zip(values.rev()))
    {
        let delta = reward + gamma * next_value - value;
        gae = delta + gamma * gae_lambda * gae;
        *adv = gae;
        *ret = gae + value;
        next_value = value;
    }

    (advantages, returns)
}
