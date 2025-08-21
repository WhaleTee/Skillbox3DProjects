using Unity.Entities;
using Unity.Mathematics;

namespace Project.Scripts.Components.Character.Abilities
{
  public struct MoveAbility : IComponentData, IEnableableComponent
  {
    public float3 Direction;
    public float Speed;
  }
}