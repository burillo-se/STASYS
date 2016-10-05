/*
 * STASYS v0.4
 *
 * (Solar Tracking Alignment SYStem)
 *
 * Published under "do whatever you want with it" license (aka public domain).
 *
 */

const string VERSION = "0.4";

public List<IMyCubeGrid> local_grids = new List<IMyCubeGrid>();
public List<IMyTerminalBlock> local_text_panels = new List<IMyTerminalBlock>();
public Dictionary<string, STASYS_Group> stasys_groups = new Dictionary<string, STASYS_Group>();

Vector3D sun_vector;
Vector3D prev_sun_vector;
Vector3D sun_normal;
float cur_max_efficiency;
float degrees_per_second;
TimeSpan time_since_last_scan;
TimeSpan time_since_last_rotation;
TimeSpan darkness_timeout;
bool init;

// state machine
Action [] states = null;

int current_state;

public class STASYS_Group {
 enum State {
  Idle,
  Search,
  Found,
  Align
 };
 enum SearchType {
  Coarse,
  Fine
 }
 enum SearchDirection {
  Lateral,
  Vertical
 }
 enum DirectionSearchState {
  MoveToStart,
  MoveToEnd,
  MoveToMax
 }
 class SearchState {
  public DirectionSearchState state;
  public SearchType type;
  public SearchDirection direction;
  public IMyMotorBase rotor;
  public float cur_start;
  public float cur_end;
  public SearchState() {
   angles = new List<float>();
   efficiencies = new List<float>();
  }
  public float getMaxAngle() {
   int max_idx = 0;
   float max_efficiency = 0;
   for (int i = 0; i < angles.Count; i++) {
    var e = efficiencies[i];
    if (e >= max_efficiency) {
     max_efficiency = e;
     max_idx = i;
    }
   }
   return angles[max_idx];
  }
  public float getMaxEfficiency() {
   float max_efficiency = 0;
   for (int i = 0; i < angles.Count; i++) {
    var e = efficiencies[i];
    if (e >= max_efficiency) {
     max_efficiency = e;
    }
   }
   return max_efficiency;
  }
  public void addData(float efficiency, float angle) {
   angles.Add(angle);
   efficiencies.Add(efficiency);
  }
  public void clearData() {
   angles.Clear();
   efficiencies.Clear();
  }
  List<float> angles;
  List<float> efficiencies;
 }
 public string name;
 Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> panels;
 Dictionary<IMyCubeGrid, IMyMotorBase> grid_to_rotor;
 bool large_grid;
 SearchState search_state;
 State state;
 Vector3D alignment_vector;

 public STASYS_Group() {
  panels = new Dictionary<IMyCubeGrid, List<IMyTerminalBlock>>();
  grid_to_rotor = new Dictionary<IMyCubeGrid, IMyMotorBase>();
  name = "";
  large_grid = true;
  state = State.Idle;
 }

 public void processState() {
  switch (state) {
   case State.Idle:
    foreach (var pair in grid_to_rotor) {
     stopRotor(pair.Value);
    }
    break;
   case State.Search:
    doSearch();
    break;
   case State.Found:
    align();
    break;
   case State.Align:
    if (!align()) {
     state = State.Idle;
    }
    break;
   default:
    break;
  }
 }

 public void findSun() {
  if (state != State.Idle) {
   return;
  }
  // if we have high efficiency at the outself, do a fine search from the outset
  SearchType t;
  if (getEfficiency() > 0.85) {
   t = SearchType.Fine;
  } else {
   t = SearchType.Coarse;
  }
  state = State.Search;
  bool fast = t == SearchType.Coarse;
  var base_rotor = findBaseRotor(getFirstPanel()[0].CubeGrid);
  search_state = new SearchState();
  search_state.rotor = base_rotor;
  var angle = normalizeAngle(getRotorAngle(search_state.rotor));
  var offset = fast ? 90 : 20;
  var start = angle - offset;
  var target = angle + offset;
  search_state.direction = SearchDirection.Lateral;
  search_state.cur_start = start;
  search_state.cur_end = target;
  search_state.type = t;
  search_state.state = DirectionSearchState.MoveToStart;
  moveRotor(search_state.rotor, start, true);
  // at this point, we're moving to our start position
 }

 public bool foundSun() {
  return state == State.Found && !align();
 }

 public bool isIdle() {
  return state == State.Idle;
 }

 public bool scanningForSun() {
  return state == State.Search;
 }

 public void stopScan() {
  state = State.Idle;
 }

 public Vector3D getCurVector() {
  var list = getFirstPanel();
  foreach (var panel in list) {
   return panel.WorldMatrix.Forward;
  }
  throw new Exception("Something exploded");
 }

 public void setPanels(List<IMyTerminalBlock> blocks) {
  panels.Clear();
  var directions = new Dictionary<IMyCubeGrid, Vector3D>();
  foreach (var panel in blocks) {
   var dir = Vector3D.Abs(panel.WorldMatrix.Forward);
   var grid = panel.CubeGrid;
   if (directions.ContainsKey(grid)) {
    var other_dir = directions[grid];
    if (other_dir != dir) {
     throw new Exception("Misaligned panel: " + panel.CustomName);
    }
   } else {
    directions.Add(grid, dir);
   }
   if (panels.ContainsKey(grid)) {
    panels[grid].Add(panel);
   } else {
    var list = new List<IMyTerminalBlock>() {panel};
    panels.Add(grid, list);
   }
  }
 }

 public void setRotors(Dictionary<IMyMotorBase, IMyCubeGrid> rotors) {
  grid_to_rotor.Clear();
  var directions = new HashSet<Vector3D>();
  foreach (var pair in rotors) {
   var rotor = pair.Key;
   var dir = Vector3D.Abs(rotor.WorldMatrix.Up);
   if (!directions.Contains(dir) && directions.Count > 2) {
    throw new Exception("Misaligned rotor: " + rotor.CustomName);
   }
   grid_to_rotor.Add(pair.Value, rotor);
   directions.Add(dir);
   large_grid = rotor.BlockDefinition.ToString().Contains("Large");
  }
 }

 public void reset() {
  if (align(findFirstBaseRotor().WorldMatrix.Forward)) {
   state = State.Align;
  } else {
   state = State.Idle;
  }
 }

 public void setAlignment(Vector3D direction) {
  if (state != State.Idle && state != State.Align) {
   return;
  }
  state = State.Align;
  align(direction);
 }

 public bool hasSolarPanels() {
  foreach (var pair in panels) {
   foreach (var block in pair.Value) {
    if (block is IMySolarPanel) {
     return true;
    }
   }
  }
  return false;
 }

 public bool hasOxygenFarms() {
  foreach (var pair in panels) {
   foreach (var block in pair.Value) {
    if (block is IMyOxygenFarm) {
     return true;
    }
   }
  }
  return false;
 }

 public float getSolarEfficiency() {
  float cur = 0;
  float total = 0;
  foreach (var pair in panels) {
   foreach (var block in pair.Value) {
    if (!(block is IMySolarPanel)) {
     continue;
    }
    var panel = block as IMySolarPanel;
    float max = large_grid ? 0.120f : 0.30f;
    total += max;
    cur += panel.MaxOutput;
   }
  }
  return total == 0 ? 0 : cur / total;
 }

 public float getSolarOutput() {
  float cur = 0;
  foreach (var pair in panels) {
   foreach (var block in pair.Value) {
    if (!(block is IMySolarPanel)) {
     continue;
    }
    var panel = block as IMySolarPanel;
    cur += panel.MaxOutput;
   }
  }
  return cur * 1000f; // output in kiloWatts
 }

 public float getOxygenEfficiency() {
  float cur = 0;
  float total = 0;
  foreach (var pair in panels) {
   foreach (var block in pair.Value) {
    if (!(block is IMyOxygenFarm)) {
     continue;
    }
    var farm = block as IMyOxygenFarm;
    float max = 1f;
    total += max;
    cur += farm.GetOutput();
   }
  }
  return total == 0 ? 0 : cur / total;
 }

 public float getOxygenOutput() {
  float cur = 0;
  foreach (var pair in panels) {
   foreach (var block in pair.Value) {
    if (!(block is IMyOxygenFarm)) {
     continue;
    }
    var farm = block as IMyOxygenFarm;
    cur += farm.GetOutput() * 1.8f;
   }
  }
  return cur; // output in liters
 }

 /*
  * Internals
  */
 private void doSearch() {
  var angle = getRotorAngle(search_state.rotor);
  var efficiency = getEfficiency();
  if (search_state.state == DirectionSearchState.MoveToStart &&
      search_state.cur_start == angle) {
   // we've reached our start, so start moving towards our target
   search_state.state = DirectionSearchState.MoveToEnd;
   bool fast = search_state.type == SearchType.Coarse;
   moveRotor(search_state.rotor, search_state.cur_end, fast);
   search_state.clearData();
  } else if (search_state.state == DirectionSearchState.MoveToEnd &&
             search_state.cur_end == angle) {
   // we've reached our end, so get max angle and start moving towards it
   search_state.state = DirectionSearchState.MoveToMax;
   var target = search_state.getMaxAngle();
   search_state.cur_start = angle;
   search_state.cur_end = target;
   moveRotor(search_state.rotor, target, true);
  } else if (search_state.state == DirectionSearchState.MoveToMax &&
             search_state.cur_end == angle) {
   // we've reached our maximum - now it's time to either switch to different
   // rotor, or do a fine search

   // first, try a different rotor
   bool done = false;
   if (search_state.direction == SearchDirection.Lateral) {
    var new_rotor = grid_to_rotor[getFirstPanel()[0].CubeGrid];
    if (new_rotor != search_state.rotor) {
     stopRotor(search_state.rotor);
     search_state.rotor = new_rotor;
     angle = normalizeAngle(getRotorAngle(search_state.rotor));
     var offset = search_state.type == SearchType.Coarse ? 90 : 20;
     var start = angle - offset;
     var target = angle + offset;
     search_state.direction = SearchDirection.Vertical;
     search_state.cur_start = start;
     search_state.cur_end = target;
     search_state.state = DirectionSearchState.MoveToStart;
     moveRotor(search_state.rotor, start, true);
     return;
    } else {
     done = true;
    }
   }
   if (search_state.direction == SearchDirection.Vertical || done) {
    // there are no other rotors, so either switch to next scanning mode, or we're done
    if (search_state.type == SearchType.Coarse) {
     // if we didn't find anything, stop
     var e = search_state.getMaxEfficiency();
     if (e < 0.10) {
      reset();
      return;
     }
     // update our rotor
     stopRotor(search_state.rotor);
     search_state.rotor = findBaseRotor(getFirstPanel()[0].CubeGrid);
     angle = getRotorAngle(search_state.rotor);
     var offset = 20;
     var start = angle - offset;
     var target = angle + offset;
     search_state.direction = SearchDirection.Lateral;
     search_state.type = SearchType.Fine;
     search_state.cur_start = start;
     search_state.cur_end = target;
     search_state.state = DirectionSearchState.MoveToStart;
     moveRotor(search_state.rotor, start, true);
    } else {
     // we're done, so get current vector, align everything and report for duty
     align(getCurVector());
     state = State.Found;
    }
   }
  }
  search_state.addData(efficiency, angle);
 }

 private IMyMotorBase findFirstBaseRotor() {
  foreach (var pair in grid_to_rotor) {
   var rotor = pair.Value;
   if (!grid_to_rotor.ContainsKey(rotor.CubeGrid)) {
    return rotor;
   }
  }
  return null;
 }

 private IMyMotorBase findBaseRotor(IMyCubeGrid grid) {
  var queue = new Queue<IMyMotorBase>();
  queue.Enqueue(grid_to_rotor[grid]);
  while (true) {
   var rotor = queue.Dequeue();
   if (grid_to_rotor.ContainsKey(rotor.CubeGrid)) {
    queue.Enqueue(grid_to_rotor[rotor.CubeGrid]);
   } else {
    return rotor;
   }
  }
  throw new Exception("Disconnected rotor grids found");
 }

 private float getRotorAngle(IMyTerminalBlock b) {
  var angle_regex = new System.Text.RegularExpressions.Regex("([\\-]?\\d+)");
  var cur_match = angle_regex.Match(b.DetailedInfo);
  if (!cur_match.Success) {
   return 0;
  }
  var val = float.Parse(cur_match.Groups[1].Value);
  return val;
 }

 private float getAngle(Vector3D normal, Vector3D src, Vector3D dst) {
  var src_dot = Vector3D.Dot(src, normal);
  var dst_dot = Vector3D.Dot(dst, normal);
  var src_dst_dot = Vector3D.Dot(src, dst);

  // sometimes, src or dst vector is orthogonal to the plane, or they are parallel
  if (Math.Abs(src_dot) > 0.99 || Math.Abs(dst_dot) > 0.99 || Math.Abs(src_dst_dot) > 0.99) {
   return 0f;
  }
  var src_proj = Vector3D.Normalize(src - src_dot * normal);
  var dst_proj = Vector3D.Normalize(dst - dst_dot * normal);

  var dot = Vector3D.Dot(src_proj, dst_proj);
  var cross = Vector3D.Normalize(Vector3D.Cross(src_proj, dst_proj));
  var cross_dot = Vector3D.Dot(normal, cross);
  var rad = Math.Acos(dot);
  var deg = (float) Math.Round(MathHelper.ToDegrees(rad));
  var result = dot < 0 ? -(180 - deg) : deg;
  result = cross_dot > 0 ? result : -result;
  return result;
 }

 private float normalizeAngle(float angle) {
  while (angle > 180) {
   angle -= 180;
  }
  while (angle < -180) {
   angle += 180;
  }
  return angle;
 }

 private float angleDiff(float a1, float a2) {
  var n_a1 = normalizeAngle(a1);
  var n_a2 = normalizeAngle(a2);
  var bigger = Math.Max(n_a1, n_a2);
  var smaller = Math.Min(n_a1, n_a2);
  var diff = bigger - smaller;
  diff = Math.Min(diff, smaller + 180 - bigger);
  return diff;
 }

 private void moveRotor(IMyMotorBase rotor, float target, bool fast = false) {
  var cur_angle = getRotorAngle(rotor);
  bool right = target > cur_angle;
  float speed = fast ? 2f : 0.4f;
  rotor.SetValue("Velocity", right ? speed : -speed);
  rotor.SetValue("UpperLimit", right ? target : cur_angle);
  rotor.SetValue("LowerLimit", right ? cur_angle : target);
  rotor.SetValue("Weld speed", 20f);
 }

 private void stopRotor(IMyMotorBase rotor) {
  rotor.SetValue("Velocity", 0f);
  rotor.SetValue("Weld speed", 2f);
 }

 private bool moveRotor(IMyMotorBase rotor, Vector3D vec) {
  var normal = rotor.WorldMatrix.Up;
  var offset = getAngle(normal, vec, alignment_vector);
  var cur_angle = getRotorAngle(rotor);
  var target = normalizeAngle(cur_angle - offset);
  if (angleDiff(target, cur_angle) < 1) {
   stopRotor(rotor);
   return false;
  } else {
   moveRotor(rotor, target, true);
   return true;
  }
 }

 private bool alignPanelsByRotors() {
  bool result = false;
  foreach (var pair in panels) {
   foreach (var b in pair.Value) {
    Vector3D dir = b.WorldMatrix.Forward;
    var rotor = grid_to_rotor[b.CubeGrid];
    result |= moveRotor(rotor, dir);
    break;
   }
  }
  return result;
 }

 private bool alignBaseRotors() {
  bool result = false;
  foreach (var pair in panels) {
   foreach (var b in pair.Value) {
    var rotor = findBaseRotor(b.CubeGrid);
    Vector3D dir = b.WorldMatrix.Forward;
    result |= moveRotor(rotor, dir);
    break;
   }
  }
  return result;
 }

 private bool align() {
  bool result;
  result = alignPanelsByRotors();
  result |= alignBaseRotors();
  return result;
 }

 private bool align(Vector3D direction) {
  alignment_vector = direction;
  return align();
 }
 private List<IMyTerminalBlock> getFirstPanel() {
  foreach (var pair in panels) {
   return pair.Value;
  }
  throw new Exception("No panels");
 }

 private float getEfficiency() {
  float cur = 0;
  float total = 0;
  var list = getFirstPanel();
  foreach (var block in list) {
   if (block is IMySolarPanel) {
    var panel = block as IMySolarPanel;
    float max = large_grid ? 0.120f : 0.30f;
    total += max;
    cur += panel.MaxOutput;
   } else {
    var farm = block as IMyOxygenFarm;
    float max = 1.80f;
    total += max;
    cur += farm.GetOutput();
   }
  }
  return total == 0 ? 0 : cur / total;
 }
}

/*
 * Graph-based grid locality code transplanted from BARABAS.
 */

// grid graph edge class, represents a connection point between two grids.
public class Edge < T > {
 public T src {get; set;}
 public T dst {get; set;}
}

// comparer for graph edges - the way the comparison is done means the edges are
// bidirectional - meaning, it doesn't matter which grid is source and which
// grid is destination, they will be equal as far as comparison is concerned.
public class EdgeComparer < T > : IEqualityComparer < Edge < T > > {
 public int GetHashCode(Edge < T > e) {
  // multiply src hashcode by dst hashcode - multiplication is commutative, so
  // result will be the same no matter which grid was source or destination
  return e.src.GetHashCode() * e.dst.GetHashCode();
 }
 public bool Equals(Edge < T > e1, Edge < T > e2) {
  if (e1.src.Equals(e2.src) && e1.dst.Equals(e2.dst)) {
   return true;
  }
  if (e1.src.Equals(e2.dst) && e1.dst.Equals(e2.src)) {
   return true;
  }
  return false;
 }
}

// our grid graph
public class Graph < T > {
 public Graph() {
  cmp = new EdgeComparer < T >();
  v_edges = new Dictionary < T, HashSet < Edge < T > > >();
  r_edges = new HashSet < Edge < T > >(cmp);
 }

 // add an edge to the graph
 public void addEdge(T src, T dst, bool is_remote) {
  var t = new Edge<T>();
  t.src = src;
  t.dst = dst;

  // remote edges don't need to be added to local list of edges
  if (is_remote) {
   r_edges.Add(t);
   return;
  }

  // add edge to list of per-vertex edges
  HashSet < Edge < T > > hs_src, hs_dst;
  if (!v_edges.TryGetValue(src, out hs_src)) {
   hs_src = new HashSet < Edge < T > >(cmp);
   v_edges.Add(src, hs_src);
  }
  if (!v_edges.TryGetValue(dst, out hs_dst)) {
   hs_dst = new HashSet < Edge < T > >(cmp);
   v_edges.Add(dst, hs_dst);
  }
  hs_src.Add(t);
  hs_dst.Add(t);
 }

 // get all grids that are local to source grid (i.e. all grids connected by
 // rotors or pistons)
 public List < T > getGridRegion(T src) {
  // if there never was a local edge from/to this grid, it's by definition
  // the only grid in this region
  if (!v_edges.ContainsKey(src)) {
   return new List < T >() {src};
  }
  // otherwise, gather all vertices in this region
  var region = new List<T>();
  var seen = new HashSet<T>();
  var next = new Queue<T>();
  next.Enqueue(src);
  while (next.Count != 0) {
   var g = next.Dequeue();
   if (!seen.Contains(g)) {
    var edges = v_edges[g];
    foreach (var edge in edges) {
     next.Enqueue(edge.src);
     next.Enqueue(edge.dst);
    }
    seen.Add(g);
    region.Add(g);
   }
  }
  return region;
 }

 // this must be called after adding all edges. what this does is, it removes
 // edges that aren't supposed to be there. For example, if you have grids
 // A, B, C, local edges A->B and B->C, and a remote edge C->A, there is a path
 // from C to A through local edges, so the remote edge should not count as an
 // actual "remote" edge, and therefore should be removed.
 public void validateGraph() {
  var to_remove = new HashSet < Edge <T> >(cmp);
  var seen = new HashSet<T>();
  foreach (var edge in r_edges) {
   var next = new Queue<T>();
   next.Enqueue(edge.src);
   next.Enqueue(edge.dst);
   while (next.Count != 0) {
    var g = next.Dequeue();
    if (!seen.Contains(g)) {
     var region = new HashSet<T>(getGridRegion(g));
     seen.UnionWith(region);
     // find any edges that are completely inside this region, and remove them
     foreach (var e in r_edges) {
      if (region.Contains(e.src) && region.Contains(e.dst)) {
       to_remove.Add(e);
      }
     }
    }
   }
  }
  foreach (var e in to_remove) {
   r_edges.Remove(e);
  }
 }

 // our comparer to use with all sets
 EdgeComparer < T > cmp;
 // list of all edges
 HashSet < Edge < T > > r_edges;
 // dictionaries of edges for each vertex
 Dictionary < T, HashSet < Edge < T > > > v_edges;
}

/*
 * Grid stuff
 */

IMySlimBlock slimBlock(IMyTerminalBlock b) {
 return b.CubeGrid.GetCubeBlock(b.Position);
}

bool blockExists(IMyTerminalBlock b) {
 return b.CubeGrid.CubeExists(b.Position);
}

// find which grid has a block at world_pos, excluding "self"
IMyCubeGrid findGrid(Vector3D w_p, IMyCubeGrid self, List < IMyCubeGrid > grids) {
 foreach (var g in grids) {
  if (g == self) {
   continue;
  }
  var pos = g.WorldToGridInteger(w_p);
  if (g.CubeExists(pos)) {
   return g;
  }
 }
 return null;
}

IMyCubeGrid getConnectedGrid(IMyShipConnector c) {
 if (!c.IsConnected) {
  return null;
 }
 // skip connectors connecting to the same grid
 var o = c.OtherConnector;
 if (o.CubeGrid == c.CubeGrid) {
  return null;
 }
 return o.CubeGrid;
}

IMyCubeGrid getConnectedGrid(IMyMotorBase r, List < IMyCubeGrid > grids) {
 if (!r.IsAttached) {
  return null;
 }
 var pos = r.Position;
 var or = r.Orientation;
 var dir = new Vector3I(0, 1, 0);
 Matrix m;
 or.GetMatrix(out m);
 Vector3I.Transform(ref dir, ref m, out dir);
 var w_p = r.CubeGrid.GridIntegerToWorld(pos + dir);
 return findGrid(w_p, r.CubeGrid, grids);
}

IMyCubeGrid getConnectedGrid(IMyPistonBase p, List < IMyCubeGrid > grids) {
 if (!p.IsAttached) {
  return null;
 }
 var pos = p.Position;
 var or = p.Orientation;
 bool is_large = p.BlockDefinition.ToString().Contains("Large");
 var up = (int) Math.Round(p.CurrentPosition / (is_large ? 2.5 : 0.5));
 var dir = new Vector3I(0, 2 + up, 0);
 Matrix m;
 or.GetMatrix(out m);
 Vector3I.Transform(ref dir, ref m, out dir);
 var w_p = p.CubeGrid.GridIntegerToWorld(pos + dir);
 return findGrid(w_p, p.CubeGrid, grids);
}

// getting local grids is not trivial, we're basically building a graph of all
// grids and figure out which ones are local to us. we are also populating
// object lists in the meantime
List < IMyCubeGrid > getLocalGrids(bool force_update = false) {
 if (local_grids != null && !force_update) {
  return local_grids;
 }

 // piston and rotor lists are local, we don't need them once we're done
 var blocks = new List<IMyTerminalBlock>();
 var pistons = new List < IMyTerminalBlock > ();
 var rotors = new List < IMyTerminalBlock > ();
 var connectors = new List < IMyTerminalBlock > ();
 var grids_set = new HashSet<IMyCubeGrid>();

 // get all blocks that are accessible to GTS
 GridTerminalSystem.GetBlocks(blocks);

 // for each block, get its grid, store data for this grid, and populate respective
 // object list if it's one of the objects we're interested in
 foreach (var b in blocks) {
  if (slimBlock(b) == null) {
   continue;
  }
  grids_set.Add(b.CubeGrid);

  // fill all lists
  if (b is IMyShipConnector) {
   connectors.Add(b);
  } else if (b is IMyPistonBase) {
   pistons.Add(b);
  } else if (b is IMyMotorBase) {
   rotors.Add(b);
  }
 }
 // free the memory!
 blocks.Clear();
 var grids = new List<IMyCubeGrid>(grids_set);

 // now, build a graph of all grids
 var gr = new Graph<IMyCubeGrid>();

 // first, go through all pistons
 foreach (IMyPistonBase p in pistons) {
  var connected_grid = getConnectedGrid(p, grids);

  if (connected_grid != null) {
   // grids connected to pistons are local to their source
   gr.addEdge(p.CubeGrid, connected_grid, false);
  }
 }

 // do the same for rotors
 foreach (IMyMotorBase rotor in rotors) {
  var connected_grid = getConnectedGrid(rotor, grids);

  if (connected_grid != null) {
   // grids connected to locals are local to their source
   gr.addEdge(rotor.CubeGrid, connected_grid, false);
  }
 }

 // do the same for connectors
 foreach (IMyShipConnector c in connectors) {
  var connected_grid = getConnectedGrid(c);

  if (connected_grid != null) {
   // grids connected to connectors belong to a different ship
   gr.addEdge(c.CubeGrid, connected_grid, true);
  }
 }

 // make sure we remove all unnecessary edges from the graph
 gr.validateGraph();

 // now, get our actual local grid
 local_grids = gr.getGridRegion(Me.CubeGrid);

 return local_grids;
}

public void filterLocalGrid(List < IMyTerminalBlock > blocks) {
 var grids = getLocalGrids();
 for (int i = blocks.Count - 1; i >= 0; i--) {
  var block = blocks[i];
  var grid = block.CubeGrid;
  if (!grids.Contains(grid)) {
   blocks.RemoveAt(i);
  }
 }
}

public void filterLocalGrid<T>(List < IMyTerminalBlock > blocks) {
 var grids = getLocalGrids();
 for (int i = blocks.Count - 1; i >= 0; i--) {
  var block = blocks[i];
  var grid = block.CubeGrid;
  if (!grids.Contains(grid) || !(block is T)) {
   blocks.RemoveAt(i);
  }
 }
}

/*
 * Misc functions
 */
// code duplication ftw! that's what you get when you don't have statics...
float getAngle(Vector3D normal, Vector3D src, Vector3D dst) {
 var src_dot = Vector3D.Dot(src, normal);
 var dst_dot = Vector3D.Dot(dst, normal);
 var src_dst_dot = Vector3D.Dot(src, dst);

 // sometimes, src or dst vector is orthogonal to the plane, or they are parallel
 if (Math.Abs(src_dot) > 0.99 || Math.Abs(dst_dot) > 0.99 || Math.Abs(src_dst_dot) > 0.99) {
  return 0f;
 }
 var src_proj = Vector3D.Normalize(src - src_dot * normal);
 var dst_proj = Vector3D.Normalize(dst - dst_dot * normal);

 var dot = Vector3D.Dot(src_proj, dst_proj);
 var cross = Vector3D.Normalize(Vector3D.Cross(src_proj, dst_proj));
 var cross_dot = Vector3D.Dot(normal, cross);
 var rad = Math.Acos(dot);
 var deg = (float) Math.Round(MathHelper.ToDegrees(rad));
 var result = dot < 0 ? -(180 - deg) : deg;
 result = cross_dot > 0 ? result : -result;
 return result;
}

string getUnitStr(float v) {
 var pwrs = "kMGTPEZY";
 int pwr_idx = 0;
 while (v >= 1000) {
  v /= 1000;
  pwr_idx++;
 }
 if (v >= 100)
  return String.Format("{0:0}", v) + pwrs[pwr_idx];
 else if (v >= 10)
  return String.Format("{0:0.0}", v) + pwrs[pwr_idx];
 else
  return String.Format("{0:0.00}", v) + pwrs[pwr_idx];
}

void showOnHud(IMyTerminalBlock b) {
 if (b.GetProperty("ShowOnHUD") != null) {
  b.SetValue("ShowOnHUD", true);
 }
}

void hideFromHud(IMyTerminalBlock b) {
 if (b.GetProperty("ShowOnHUD") != null) {
  b.SetValue("ShowOnHUD", false);
 }
}

STASYS_Group updateFromGroup(List<IMyTerminalBlock> blocks, STASYS_Group g) {
 // we have to get all rotors first
 var rotors = new Dictionary<IMyMotorBase, IMyCubeGrid>();
 var panels = new List<IMyTerminalBlock>();
 for (int i = blocks.Count - 1; i >= 0; i--) {
  var block = blocks[i];
  if (block is IMyMotorBase && !(block is IMyMotorSuspension)) {
   var rotor = block as IMyMotorBase;
   var grid = getConnectedGrid(rotor, getLocalGrids());
   if (grid == null) {
    throw new Exception(rotor.CustomName + " is not connected to anything");
   }
   rotors.Add(rotor, grid);
  } else if (block is IMyOxygenFarm || block is IMySolarPanel) {
   panels.Add(block);
  } else {
   throw new Exception("Unexpected block: " + block.CustomName);
  }
 }
 g.setRotors(rotors);
 g.setPanels(panels);
 return g;
}

/*
 * States
 */

void s_processStates() {
 if (time_since_last_rotation < darkness_timeout) {
  return;
 }
 bool scan = false;
 if (sun_normal != Vector3D.Zero && time_since_last_rotation > new TimeSpan(0, 0, 15)) {
  var qt = Quaternion.CreateFromAxisAngle(sun_normal, MathHelper.ToRadians(degrees_per_second * 15f));
  sun_vector = Vector3D.Transform(sun_vector, qt);
  time_since_last_rotation = TimeSpan.Zero;
 } else if (!init ||
            (sun_normal != Vector3D.Zero && time_since_last_scan > new TimeSpan(0, 6, 0)) ||
            (sun_normal == Vector3D.Zero && time_since_last_scan > new TimeSpan(0, 1, 0))) {
  time_since_last_scan = TimeSpan.Zero;
  cur_max_efficiency = 0;
  if (prev_sun_vector != Vector3D.Zero) {
   sun_normal = Vector3D.Normalize(Vector3D.Cross(sun_vector, prev_sun_vector));
   // get angle between prev and cur sun vectors
   var angle = getAngle(sun_normal, prev_sun_vector, sun_vector);
   degrees_per_second = angle / 180f;
   prev_sun_vector = Vector3D.Zero;
  } else {
   prev_sun_vector = sun_vector;
   scan = true;
  }
 }
 bool all_dark = true;
 foreach (var pair in stasys_groups) {
  var g = pair.Value;
  g.processState();
  if (scan == true) {
   g.findSun();
   init = true;
  }
  var e = Math.Max(g.getSolarEfficiency(), g.getOxygenEfficiency());
  if (g.isIdle() && e > 0.05) {
   all_dark = false;
  }
  if (g.foundSun()) {
   var max_v = g.getCurVector();
   g.stopScan();
   if (e > 0.10 && e > cur_max_efficiency) {
    sun_vector = max_v;
    cur_max_efficiency = e;
   }
  } else if (sun_vector != Vector3D.Zero) {
   g.setAlignment(sun_vector);
  }
 }
 if (all_dark) {
  time_since_last_rotation = TimeSpan.Zero;
  darkness_timeout = new TimeSpan(0, 5, 0);
 }
}

void s_displayStats() {
 if (local_text_panels.Count == 0) {
  return;
 }
 var sb = new StringBuilder();
 float o_e = 0f, s_e = 0f;
 int o_n = 0, s_n = 0;
 float o_o = 0f, s_o = 0f;
 foreach (var pair in stasys_groups) {
  var g = pair.Value;
  if (g.hasOxygenFarms()) {
   o_n++;
   o_e += g.getOxygenEfficiency();
   o_o += g.getOxygenOutput();
  }
  if (g.hasSolarPanels()) {
   s_n++;
   s_e += g.getSolarEfficiency();
   s_o += g.getSolarOutput();
  }
 }
 o_e /= o_n == 0 ? 1 : o_n;
 s_e /= s_n == 0 ? 1 : s_n;

 sb.AppendLine(String.Format("STASYS v{0}", VERSION));
 sb.AppendLine(String.Format("Groups under control: {0}", stasys_groups.Count));
 sb.Append("Oxygen farm efficiency: ");
 if (o_n > 0) {
  sb.AppendLine(String.Format("{0:0.0}% ({1:0} L)", o_e * 100f, o_o));
 } else {
  sb.AppendLine("N/A");
 }
 sb.Append("Solar panel efficiency: ");
 if (s_n > 0) {
  sb.AppendLine(String.Format("{0:0.0}% ({1}W)", s_e * 100f, getUnitStr(s_o)));
 } else {
  sb.AppendLine("N/A");
 }
 foreach (IMyTextPanel panel in local_text_panels) {
  panel.WritePublicTitle("STASYS Notification");
  panel.WritePublicText(sb.ToString());
  panel.ShowPublicTextOnScreen();
 }
}

void s_refreshGroups() {
  var tmp_groups = new Dictionary<string, STASYS_Group>();
  var groups = new List<IMyBlockGroup>();
  GridTerminalSystem.GetBlockGroups(groups);
  for (int i = 0; i < groups.Count; i++) {
   var group = groups[i];
   // skip groups we don't want
   if (!group.Name.StartsWith("STASYS") || group.Name == "STASYS Notify") {
    continue;
   }
   var blocks = new List<IMyTerminalBlock>();
   group.GetBlocks(blocks);
   filterLocalGrid(blocks);
   if (blocks.Count == 0) {
    // this group is from a foreign grid
    continue;
   }
   try {
    var name = group.Name;
    STASYS_Group g;
    if (stasys_groups.ContainsKey(name)) {
     g = stasys_groups[name];
     updateFromGroup(blocks, g);
    } else {
     g = new STASYS_Group();
     updateFromGroup(blocks, g);
     g.name = name;
     g.reset();
    }
    tmp_groups.Add(name, g);
   } catch (Exception e) {
    Echo(group.Name + ": " + e.Message);
   }
  }
  stasys_groups = tmp_groups;
}

void s_refreshNotification() {
 var blocks = new List < IMyTerminalBlock > ();
 var g = GridTerminalSystem.GetBlockGroupWithName("STASYS Notify");
 if (g != null) {
  g.GetBlocks(blocks);
  filterLocalGrid < IMyTextPanel > (blocks);
 }
 local_text_panels = blocks;
}

void s_refreshGrids() {
 getLocalGrids(true);
}

int[] state_cycle_counts;
int cycle_count;

bool canContinue() {
 bool hasHeadroom = false;
 bool isFirstRun = state_cycle_counts[current_state] == 0;
 var prev_state = current_state == 0 ? states.Length - 1 : current_state - 1;
 var next_state = (current_state + 1) % states.Length;
 var cur_i = Runtime.CurrentInstructionCount;

 // now store how many cycles we've used during this iteration
 state_cycle_counts[current_state] = cur_i - cycle_count;

 var last_cycle_count = state_cycle_counts[next_state];

 // if we have enough headroom (we want no more than 80% cycle/method count)
 int projected_cycle_count = cur_i + last_cycle_count;
 Decimal cycle_percentage = (Decimal) projected_cycle_count / Runtime.MaxInstructionCount;

 // to speed up initial run, keep 40% headroom for next states
 bool initRunCycleHeadroom = isFirstRun && cycle_percentage <= 0.4M;

 bool runCycleHeadroom = !isFirstRun && cycle_percentage <= 0.8M;

 if (initRunCycleHeadroom || runCycleHeadroom) {
  hasHeadroom = true;
 }

 // advance current state and store IL count values
 current_state = next_state;
 cycle_count = cur_i;

 return hasHeadroom;
}

void ILReport(int states_executed) {
 string il_str = String.Format("IL Count: {0}/{1} ({2:0.0}%)",
    Runtime.CurrentInstructionCount,
    Runtime.MaxInstructionCount,
    (Decimal) Runtime.CurrentInstructionCount / Runtime.MaxInstructionCount * 100M);
 Echo(String.Format("States executed: {0}", states_executed));
 Echo(il_str);
}

public void Save() {
}

public Program() {
 states = new Action [] {
  s_refreshGrids,
  s_refreshGroups,
  s_refreshNotification,
  s_processStates,
  s_displayStats,
 };
 Me.SetCustomName("STASYS CPU");
 hideFromHud(Me);
 current_state = 0;
 sun_vector = new Vector3D();
 sun_normal = new Vector3D();
 cur_max_efficiency = 0f;
 time_since_last_scan = TimeSpan.Zero;
 time_since_last_rotation = TimeSpan.Zero;
 state_cycle_counts = new int[states.Length];
 init = false;
}

void Main() {
 time_since_last_scan += Runtime.TimeSinceLastRun;
 time_since_last_rotation += Runtime.TimeSinceLastRun;
 Echo(String.Format("STASYS version {0}", VERSION));
 int num_states = 0;
 cycle_count = 0;
 do {
  try {
   states[current_state]();
  } catch (Exception e) {
   Me.SetCustomName("STASYS Exception");
   showOnHud(Me);
   Echo(e.StackTrace);
   throw;
  }
  num_states++;
 } while (canContinue() && num_states < states.Length);
 Echo(String.Format("Groups under control: {0}", stasys_groups.Count));

 ILReport(num_states);
}
