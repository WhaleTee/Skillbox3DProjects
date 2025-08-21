using System;
using Unity.Entities;

namespace Project.Scripts.BasicController.Misc
{
  [Serializable]
  public struct Vehicle : IComponentData
  {
    public float MaxSpeed;
    public float Acceleration;
    public float MaxRotationSpeed;
    public float RotationAcceleration;

    public float WheelFriction;
    public float WheelRollResistance;
    public float RotationDamping;

    public float WheelSuspensionDistance;
    public float WheelSuspensionStrength;
  }

  public struct VehicleWheels : IBufferElementData
  {
    public Entity MeshEntity;
    public Entity CollisionEntity;
  }
}