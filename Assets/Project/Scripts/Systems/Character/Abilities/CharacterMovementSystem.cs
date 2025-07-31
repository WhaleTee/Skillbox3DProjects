using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[UpdateAfter(typeof(CharacterDashSystem))]
[BurstCompile]
public partial struct CharacterMovementSystem : ISystem
{
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        var dt = SystemAPI.Time.DeltaTime;
        new CharacterMovementJob { DeltaTime = dt }.Schedule();
    }
}

[BurstCompile]
public partial struct CharacterMovementJob : IJobEntity
{
    public float DeltaTime;
    
    private void Execute(ref LocalTransform transform, in MoveAbility ability)
    {
        if (ability.Direction.Equals(float3.zero)) return;

        transform.Position += ability.Direction * ability.Speed * DeltaTime;
    }
}