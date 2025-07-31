public interface UpdateContext : OnSystemCreate, OnSystemUpdate { }

public interface UpdateContext<in P1, in P2> : OnSystemCreate, OnSystemUpdate<P1, P2> { }