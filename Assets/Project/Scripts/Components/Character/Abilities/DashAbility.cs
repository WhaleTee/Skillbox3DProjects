using Unity.Entities;

namespace Project.Scripts.Components.Character.Abilities
{
  public struct DashAbility : IComponentData, IEnableableComponent
  {
    public float RemainingTime;
    public float CooldownLeft;
  }
}