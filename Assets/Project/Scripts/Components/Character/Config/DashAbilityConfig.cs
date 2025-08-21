using Unity.Entities;

namespace Project.Scripts.Components.Character.Config
{
  public struct DashAbilityConfig : IComponentData
  {
    public float Speed;
    public float Duration;
    public float Cooldown;
  }
}