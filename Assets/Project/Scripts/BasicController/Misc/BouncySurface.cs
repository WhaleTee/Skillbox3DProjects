using System;
using Unity.Entities;

namespace Project.Scripts.BasicController.Misc
{
  [Serializable]
  public struct BouncySurface : IComponentData
  {
    public float BounceEnergyMultiplier;
  }
}