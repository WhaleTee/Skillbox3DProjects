using System;
using Unity.Entities;
using Unity.Mathematics;

[Serializable]
public struct BasicCharacterControl : IComponentData {
  public float3 moveVector;
  public bool jump;
}