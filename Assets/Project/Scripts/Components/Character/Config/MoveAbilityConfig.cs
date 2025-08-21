using Unity.Entities;

namespace Project.Scripts.Components.Character.Config
{
  public struct MoveAbilityConfig : IComponentData
  {
    public float WalkSpeed;
    public float RunSpeed;
  }
}