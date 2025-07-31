using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.CharacterController;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Transforms;

[UpdateInGroup(typeof(KinematicCharacterPhysicsUpdateGroup))]
[BurstCompile]
public partial struct CharacterPhysicsUpdateSystem : ISystem {
  private EntityQuery characterQuery;
  private KinematicCharacterContext kinematicCharacterContext;
  private KinematicCharacterUpdateContext baseContext;

  [BurstCompile]
  public void OnCreate(ref SystemState state) {
    characterQuery = KinematicCharacterUtilities.GetBaseCharacterQueryBuilder()
    .WithAll<BasicCharacterProperties, BasicCharacterControl>()
    .Build(ref state);

    kinematicCharacterContext = new KinematicCharacterContext();
    baseContext = new KinematicCharacterUpdateContext();

    kinematicCharacterContext.OnSystemCreate(ref state);
    baseContext.OnSystemCreate(ref state);

    state.RequireForUpdate(characterQuery);
    state.RequireForUpdate<PhysicsWorldSingleton>();
  }

  [BurstCompile]
  public void OnDestroy(ref SystemState state) { }

  [BurstCompile]
  public void OnUpdate(ref SystemState state) {
    kinematicCharacterContext.OnSystemUpdate(ref state);
    baseContext.OnSystemUpdate(ref state, SystemAPI.Time, SystemAPI.GetSingleton<PhysicsWorldSingleton>());

    var job = new PhysicsUpdateJob { context = kinematicCharacterContext, baseContext = baseContext };
    job.ScheduleParallel();
  }

  [BurstCompile]
  public partial struct PhysicsUpdateJob : IJobEntity, IJobEntityChunkBeginEnd {
    public KinematicCharacterContext context;
    public KinematicCharacterUpdateContext baseContext;

    private void Execute(
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

      var processor = new BasicKinematicCharacterPhysicsUpdateProcessor(characterComponent, ref kinematicCharacter);
      
      var physicsStepsExecutor = new KinematicCharacterPhysicsUpdateExecutor {
        physicsUpdateProcessor = processor,
        baseContext = baseContext,
        context = context,
        characterData = kinematicCharacter,
        stepAndSlopeParameters = characterComponent.ValueRW.stepAndSlopeHandling,
        gravity = characterComponent.ValueRO.gravity
      };

      var physicsIntermediateStepHandlerIntermediateStepHandler = new KinematicCharacterPhysicsUpdateStepHandler
      {
        processor = processor,
        baseContext = baseContext,
        context = context,
        characterData = kinematicCharacter,
        characterControl = characterControl,
        characterProperties = characterComponent
      };
      
      KinematicCharacterPhysicsExecutor.Execute(physicsStepsExecutor, physicsIntermediateStepHandlerIntermediateStepHandler);
    }

    #region IJobEntityChunkBeginEnd Callbacks

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