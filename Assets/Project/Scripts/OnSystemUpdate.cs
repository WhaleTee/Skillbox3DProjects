using Unity.Entities;

namespace Project.Scripts
{
  public interface OnSystemUpdate {
    void OnSystemUpdate(ref SystemState state);
  }

  public interface OnSystemUpdate<in P1, in P2> {
    void OnSystemUpdate(ref SystemState state, P1 p1, P2 p2);
  }
}