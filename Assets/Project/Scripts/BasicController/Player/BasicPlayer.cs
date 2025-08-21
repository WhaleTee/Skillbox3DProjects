using System;
using Project.Scripts.BasicController.Misc;
using Unity.Entities;
using Unity.Mathematics;

namespace Project.Scripts.BasicController.Player
{
  [Serializable]
  public struct BasicPlayer : IComponentData
  {
    public Entity ControlledCharacter;
    public Entity ControlledCamera;
  }

  [Serializable]
  public struct BasicPlayerInputs : IComponentData
  {
    public float2 MoveInput;
    public float2 CameraLookInput;
    public float CameraZoomInput;
    public FixedInputEvent JumpPressed;
  }
}