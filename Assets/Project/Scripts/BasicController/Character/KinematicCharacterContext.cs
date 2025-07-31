using Unity.Collections;
using Unity.Entities;

public struct KinematicCharacterContext : UpdateContext {
  [ReadOnly] public ComponentLookup<BouncySurface> bouncySurfaceLookup;

  public void OnSystemCreate(ref SystemState state) => bouncySurfaceLookup = state.GetComponentLookup<BouncySurface>(true);

  public void OnSystemUpdate(ref SystemState state) => bouncySurfaceLookup.Update(ref state);
}