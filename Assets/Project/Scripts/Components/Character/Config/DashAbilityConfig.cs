using Unity.Entities;

public struct DashAbilityConfig : IComponentData
{
    public float Speed;
    public float Duration;
    public float Cooldown;
}