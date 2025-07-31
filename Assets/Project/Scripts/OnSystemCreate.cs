using Unity.Entities;

public interface OnSystemCreate {
  void OnSystemCreate(ref SystemState state);
}