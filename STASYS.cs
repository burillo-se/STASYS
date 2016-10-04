/*
 * STASYS v0.2
 *
 * (Solar Tracking Alignment SYStem)
 *
 * Published under "do whatever you want with it" license (aka public domain).
 *
 */

const string VERSION = "0.2";

public List<IMyCubeGrid> local_grids = new List<IMyCubeGrid>();
public List<IMyTerminalBlock> local_text_panels = new List<IMyTerminalBlock>();
public Dictionary<string, STASYS_Group> stasys_groups = new Dictionary<string, STASYS_Group>();
Vector3D sun_vector;
IMyTimerBlock local_timer = null;

// state machine
Action [] states = null;

int current_state;

public class STASYS_Group {
 public string name;
 private Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> panels;
 private Dictionary<IMyCubeGrid, IMyMotorBase> grid_to_rotor;
 bool large_grid;
 enum State {
  STATE_IDLE,
  STATE_SEARCH,
  STATE_FOUND,
  STATE_ALIGN
 };
 enum SearchDirection {
  DIR_LATERAL,
  DIR_VERTICAL,
  DIR_MAX
 }
 class SearchState {
  public SearchDirection cur_direction;
  public float max_search_result;
  public float max_angle;
  public IMyMotorBase cur_rotor;
  public float cur_limit;
  public bool ready;
 }
 SearchState search_state;
 State state;
 Vector3D alignment_vector;
 public STASYS_Group() {
  panels = new Dictionary<IMyCubeGrid, List<IMyTerminalBlock>>();
  grid_to_rotor = new Dictionary<IMyCubeGrid, IMyMotorBase>();
  name = "";
  large_grid = true;
  state = State.STATE_IDLE;
  search_state = new SearchState();
 }
 public void processState() {
  switch (state) {
   case State.STATE_IDLE:
    break;
   case State.STATE_SEARCH:
    doSearch();
    if (search_state.cur_direction == SearchDirection.DIR_MAX) {
     align(getCurVector());
     state = State.STATE_FOUND;
    }
    break;
   case State.STATE_FOUND:
    align();
    break;
   case State.STATE_ALIGN:
    if (!align()) {
     state = State.STATE_IDLE;
    }
    break;
  }
 }
 private void doSearch() {
  var efficiency = getEfficiency();
  if (efficiency > search_state.max_search_result) {
   search_state.max_search_result = efficiency;
   search_state.max_angle = getRotorAngle(search_state.cur_rotor);
  }
  if (!search_state.ready) {
   var angle = getRotorAngle(search_state.cur_rotor);
   if (angle == search_state.cur_limit) {
    search_state.ready = true;
    search_state.cur_limit = search_state.max_angle;
    moveRotor(search_state.cur_rotor, search_state.cur_limit, true);
   }
  } else if (getRotorAngle(search_state.cur_rotor) == search_state.cur_limit) {
   search_state.ready = false;
   search_state.cur_direction++;
   stopRotor(search_state.cur_rotor);
   if (search_state.cur_direction != SearchDirection.DIR_MAX) {
    search_state.max_angle = 0;
    search_state.max_search_result = 0;
    var new_rotor = findBaseRotor(getFirstPanel()[0].CubeGrid);
    if (new_rotor == search_state.cur_rotor) {
     search_state.cur_direction++;
    } else {
     search_state.cur_rotor = new_rotor;
     var angle = getRotorAngle(search_state.cur_rotor);
     var target = angle >= 180 ? angle - 180 : angle + 180;
     moveRotor(search_state.cur_rotor, target);
     search_state.cur_limit = target;
    }
   }
  }
 }
 public void findSun() {
  if (state != State.STATE_IDLE) {
   return;
  }
  var base_rotor = grid_to_rotor[getFirstPanel()[0].CubeGrid];
  state = State.STATE_SEARCH;
  search_state = new SearchState();
  search_state.cur_rotor = base_rotor;
  var angle = getRotorAngle(search_state.cur_rotor);
  var target = angle >= 180 ? angle - 180 : angle + 180;
  search_state.cur_limit = target;
  search_state.max_angle = 0;
  search_state.max_search_result = 0;
  search_state.ready = false;
  moveRotor(search_state.cur_rotor, target);
 }
 public bool foundSun() {
  return state == State.STATE_FOUND && !align();
 }
 private void rotate() {
  // var qt = Quaternion.CreateFromAxisAngle(search_state.up, MathHelper.ToRadians(1));
  // var result = Vector3D.Transform(alignment_vector, qt);
 }
 public Vector3D getCurVector() {
  var list = getFirstPanel();
  foreach (var panel in list) {
   return panel.WorldMatrix.Forward;
  }
  throw new Exception("Something exploded");
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
   state = State.STATE_ALIGN;
  } else {
   state = State.STATE_IDLE;
  }
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

  // sometimes, src or dst vector is orthogonal to the plane, in this case bail out
  if (src_dot > 0.99 || dst_dot > 0.99) {
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
  if (angleDiff(target, cur_angle) < 2) {
   stopRotor(rotor);
   return false;
  } else {
   moveRotor(rotor, target);
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
 public void setAlignment(Vector3D direction) {
  if (state != State.STATE_IDLE && state != State.STATE_ALIGN) {
   return;
  }
  state = State.STATE_ALIGN;
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
 public float getOxygenEfficiency() {
  float cur = 0;
  float total = 0;
  foreach (var pair in panels) {
   foreach (var block in pair.Value) {
    if (!(block is IMyOxygenFarm)) {
     continue;
    }
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
 float max_e = 0;
 Vector3D max_v;
 foreach (var pair in stasys_groups) {
  var g = pair.Value;
  try {
   g.processState();
   if (g.foundSun()) {
    max_v = g.getCurVector();
    var e = Math.Max(g.getSolarEfficiency(), g.getOxygenEfficiency());
    if (e > max_e) {
     sun_vector = max_v;
     max_e = e;
    }
   } else if (sun_vector != Vector3D.Zero) {
    g.setAlignment(sun_vector);
   }
  } catch (Exception e) {
   Echo(e.Message);
  }
 }
}

void s_displayStats() {
 if (local_text_panels.Count == 0) {
  return;
 }
 var sb = new StringBuilder();
 float o_e = 0f, s_e = 0f;
 int o_n = 0, s_n = 0;
 foreach (var pair in stasys_groups) {
  var g = pair.Value;
  if (g.hasOxygenFarms()) {
   o_n++;
   o_e += g.getOxygenEfficiency();
  }
  if (g.hasSolarPanels()) {
   s_n++;
   s_e += g.getSolarEfficiency();
  }
 }
 o_e /= o_n == 0 ? 1 : o_n;
 s_e /= s_n == 0 ? 1 : s_n;

 sb.AppendLine(String.Format("STASYS v{0}", VERSION));
 sb.AppendLine(String.Format("Groups under control: {0}", stasys_groups.Count));
 sb.AppendLine(String.Format("Dedicated timer: {0}", local_timer == null ? "no" : "yes"));
 sb.Append("Oxygen farm efficiency: ");
 if (o_n > 0) {
  sb.AppendLine(String.Format("{0:0.0}%", o_e * 100f));
 } else {
  sb.AppendLine("N/A");
 }
 sb.Append("Solar panel efficiency: ");
 if (s_n > 0) {
  sb.AppendLine(String.Format("{0:0.0}%", s_e * 100f));
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
     if (sun_vector != Vector3D.Zero) {
      g.setAlignment(sun_vector);
     } else {
      g.findSun();
     }
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

void s_refreshTimer() {
 var blocks = new List<IMyTerminalBlock>();
 GridTerminalSystem.SearchBlocksOfName("STASYS Timer", blocks);
 filterLocalGrid(blocks);
 if (blocks.Count == 0) {
  local_timer = null;
  return;
 }
 if (blocks.Count == 2) {
  throw new Exception("Multiple STASYS timers detected");
 }
 local_timer = blocks[0] as IMyTimerBlock;
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
  s_refreshTimer,
  s_processStates,
  s_displayStats,
 };
 Me.SetCustomName("STASYS CPU");
 hideFromHud(Me);
 current_state = 0;
 sun_vector = new Vector3D();
 state_cycle_counts = new int[states.Length];
}

void Main() {
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
