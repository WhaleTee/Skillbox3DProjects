using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;
using Unity.CharacterController;
using Unity.Physics.Authoring;
using UnityEngine.Serialization;

[Serializable]
public struct BasicCharacterProperties : IComponentData {
  [Header("Movement")] public float rotationSharpness;
  public float groundMaxSpeed;
  public float groundedMovementSharpness;
  public float airAcceleration;
  public float airMaxSpeed;
  public float airDrag;
  public float jumpSpeed;
  public float3 gravity;
  public bool preventAirAccelerationAgainstUngroundedHits;
  public int maxJumpsInAir;
  public BasicStepAndSlopeHandlingParameters stepAndSlopeHandling;

  [Header("Tags")] public CustomPhysicsBodyTags ignoreCollisionsTag;
  public CustomPhysicsBodyTags ignoreGroundingTag;
  public CustomPhysicsBodyTags zeroMassAgainstCharacterTag;
  public CustomPhysicsBodyTags infiniteMassAgainstCharacterTag;
  public CustomPhysicsBodyTags ignoreStepHandlingTag;

  [NonSerialized] public int currentJumpsInAir;

  public static BasicCharacterProperties GetDefault() {
    return new BasicCharacterProperties {
      rotationSharpness = 25f,
      groundMaxSpeed = 10f,
      groundedMovementSharpness = 15f,
      airAcceleration = 50f,
      airMaxSpeed = 10f,
      airDrag = 0f,
      jumpSpeed = 10f,
      gravity = math.up() * -30f,
      preventAirAccelerationAgainstUngroundedHits = true,
      maxJumpsInAir = 0,
      stepAndSlopeHandling = BasicStepAndSlopeHandlingParameters.GetDefault(),
    };
  }
}