using System;
using Unity.Entities;

namespace Project.Scripts.BasicController.Misc
{
  [Serializable]
  public struct Teleporter : IComponentData
  {
    public Entity DestinationEntity;
  }
}