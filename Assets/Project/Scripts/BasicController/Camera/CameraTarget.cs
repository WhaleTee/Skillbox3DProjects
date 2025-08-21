using System;
using Unity.Entities;

namespace Project.Scripts.BasicController.Camera
{
  [Serializable]
  public struct CameraTarget : IComponentData
  {
    public Entity TargetEntity; 
  }
}
