using Unity.Entities;

public struct DashAbility : IComponentData, IEnableableComponent
{
    public float RemainingTime;
    public float CooldownLeft;
}