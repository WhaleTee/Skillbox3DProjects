using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Project.Scripts.BasicController.Misc
{
  [Serializable]
  public struct PrefabThrower : IComponentData
  {
    public Entity PrefabEntity;
    public float3 InitialEulerAngles;
    public float ThrowForce;
  }
}