using Unity.Entities;

namespace Project.Scripts
{
  public interface OnSystemCreate {
    void OnSystemCreate(ref SystemState state);
  }
}