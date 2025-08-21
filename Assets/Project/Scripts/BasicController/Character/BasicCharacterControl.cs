using System;
using Unity.Entities;
using Unity.Mathematics;

namespace Project.Scripts.BasicController.Character
{
  [Serializable]
  public struct BasicCharacterControl : IComponentData {
    public float3 moveVector;
    public float3 cameraForward;
    public bool jump;
  }
}