using Unity.CharacterController;
using Unity.Entities;
using Unity.Physics;
using Unity.Transforms;

public struct KinematicCharacterData {

  /// <summary>
  /// The entity of the character
  /// </summary>
  public Entity entity;

  /// <summary>
  /// The local transform component of the character entity
  /// </summary>
  public RefRW<LocalTransform> localTransform;

  /// <summary>
  /// The <see cref="KinematicCharacterProperties"/> component of the character entity
  /// </summary>
  public RefRW<KinematicCharacterProperties> characterProperties;

  /// <summary>
  /// The <see cref="KinematicCharacterBody"/> component of the character entity
  /// </summary>
  public RefRW<KinematicCharacterBody> characterBody;

  /// <summary>
  /// The <see cref="physicsCollider"/> component of the character entity
  /// </summary>
  public RefRW<PhysicsCollider> physicsCollider;

  /// <summary>
  /// The <see cref="KinematicCharacterHit"/> dynamic buffer of the character entity
  /// </summary>
  public DynamicBuffer<KinematicCharacterHit> characterHitsBuffer;

  /// <summary>
  /// The <see cref="StatefulKinematicCharacterHit"/> dynamic buffer of the character entity
  /// </summary>
  public DynamicBuffer<StatefulKinematicCharacterHit> statefulHitsBuffer;

  /// <summary>
  /// The <see cref="KinematicCharacterDeferredImpulse"/> dynamic buffer of the character entity
  /// </summary>
  public DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulsesBuffer;

  /// <summary>
  /// The <see cref="KinematicVelocityProjectionHit"/> dynamic buffer of the character entity
  /// </summary>
  public DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits;
  
  public KinematicCharacterData(
    Entity entity,
    RefRW<LocalTransform> localTransform,
    RefRW<KinematicCharacterProperties> characterProperties,
    RefRW<KinematicCharacterBody> characterBody,
    RefRW<PhysicsCollider> physicsCollider,
    ref DynamicBuffer<KinematicCharacterHit> characterHitsBuffer,
    ref DynamicBuffer<StatefulKinematicCharacterHit> statefulHitsBuffer,
    ref DynamicBuffer<KinematicCharacterDeferredImpulse> deferredImpulsesBuffer,
    ref DynamicBuffer<KinematicVelocityProjectionHit> velocityProjectionHits
  ) {
    this.entity = entity;
    this.localTransform = localTransform;
    this.characterProperties = characterProperties;
    this.characterBody = characterBody;
    this.physicsCollider = physicsCollider;
    this.characterHitsBuffer = characterHitsBuffer;
    this.statefulHitsBuffer = statefulHitsBuffer;
    this.deferredImpulsesBuffer = deferredImpulsesBuffer;
    this.velocityProjectionHits = velocityProjectionHits;
  }
}