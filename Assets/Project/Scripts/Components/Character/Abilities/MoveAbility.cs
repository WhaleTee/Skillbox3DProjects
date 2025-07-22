using Unity.Entities;
using Unity.Mathematics;

public struct MoveAbility : IComponentData, IEnableableComponent
{
    public float3 Direction;
    public float Speed;
}