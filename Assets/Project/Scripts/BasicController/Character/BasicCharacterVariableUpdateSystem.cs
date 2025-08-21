using Project.Scripts.BasicController.Player;
using Project.Scripts.KinematicPhysics;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.CharacterController;
using Unity.Entities;
using Unity.Physics;
using Unity.Transforms;

namespace Project.Scripts.BasicController.Character
{
  [UpdateInGroup(typeof(SimulationSystemGroup))]
  [UpdateAfter(typeof(FixedStepSimulationSystemGroup))]
  [UpdateAfter(typeof(BasicPlayerVariableStepControlSystem))]
  [UpdateBefore(typeof(TransformSystemGroup))]
  [BurstCompile]
  public partial struct BasicCharacterVariableUpdateSystem : ISystem {
    private EntityQuery characterQuery;
    private KinematicCharacterContext context;
    private KinematicCharacterUpdateContext baseContext;

    [BurstCompile]
    public void OnCreate(ref SystemState state) {
      characterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
      .WithAll<
        BasicCharacterProperties,
        BasicCharacterControl>()
      .Build(ref state);

      context = new KinematicCharacterContext();
      context.OnSystemCreate(ref state);
      baseContext = new KinematicCharacterUpdateContext();
      baseContext.OnSystemCreate(ref state);

      state.RequireForUpdate(characterQuery);
      state.RequireForUpdate<PhysicsWorldSingleton>();
    }

    [BurstCompile]
    public void OnDestroy(ref SystemState state) { }

    [BurstCompile]
    public void OnUpdate(ref SystemState state) {
      context.OnSystemUpdate(ref state);
      baseContext.OnSystemUpdate(ref state, SystemAPI.Time, SystemAPI.GetSingleton<PhysicsWorldSingleton>());

      BasicCharacterVariableUpdateJob job = new BasicCharacterVariableUpdateJob { baseContext = baseContext, };
      job.ScheduleParallel();
    }

    [BurstCompile]
    public partial struct BasicCharacterVariableUpdateJob : IJobEntity, IJobEntityChunkBeginEnd {
      public KinematicCharacterUpdateContext baseContext;

      void Execute(
        RefRW<BasicCharacterProperties> characterComponent,
        RefRW<BasicCharacterControl> characterControl,
        RefRW<LocalTransform> localTransform,
        RefRW<KinematicCharacterProperties> characterProperties,
        RefRW<KinematicCharacterBody> characterBody,
        RefRW<PhysicsCollider> physicsCollider,
        DynamicBuffer<KinematicCharacterHit> characterHitBuffer,
        DynamicBuffer<StatefulKinematicCharacterHit> statefulHitBuffer,
        DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulseBuffer,
        DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHitBuffer,
        Entity entity
      ) {

        var kinematicCharacter = new KinematicCharacterData(
          entity,
          localTransform,
          characterProperties,
          characterBody,
          physicsCollider,
          ref characterHitBuffer,
          ref statefulHitBuffer,
          ref deferredImpulseBuffer,
          ref velocityProjectionHitBuffer
        );

        var variableUpdateHandler = new BasicCharacterVariableUpdateHandler {
          baseContext = baseContext, characterControl = characterControl, characterData = kinematicCharacter, characterProperties = characterComponent
        };

        variableUpdateHandler.HandleVariableUpdate();
      }

      #region IJobEntityChunkBeginEnd Events

      public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask) {
        baseContext.EnsureCreationOfTmpCollections();
        return true;
      }

      public void OnChunkEnd(
        in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask,
        bool chunkWasExecuted
      ) { }

      #endregion
    }
  }
}