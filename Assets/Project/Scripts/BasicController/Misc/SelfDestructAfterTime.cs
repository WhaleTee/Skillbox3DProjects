using System;
using Unity.Entities;

namespace Project.Scripts.BasicController.Misc
{
  [Serializable]
  public struct SelfDestructAfterTime : IComponentData
  {
    public float LifeTime;
    public float TimeSinceAlive;
  }
}