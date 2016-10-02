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
public List<STASYS_Group> stasys_groups = new List<STASYS_Group>();
IMyTimerBlock local_timer = null;

// state machine
Action [] states = null;

int current_state;

public class STASYS_Group {
 public string name;
 private Dictionary<Vector3D, List<IMySolarPanel>> solar_panels;
 private Dictionary<Vector3D, List<IMyOxygenFarm>> oxygen_farms;
 private Dictionary<IMyCubeGrid, Vector3D> solar_grid_vectors;
 private Dictionary<IMyCubeGrid, Vector3D> oxygen_grid_vectors;
 private Dictionary<IMyCubeGrid, Vector3D> grid_to_rotor_vector;
 private Dictionary<Vector3D, List<IMyMotorBase>> rotors;
 bool large_grid;
 public STASYS_Group() {
  solar_panels = new Dictionary<Vector3D, List<IMySolarPanel>>();
  oxygen_farms = new Dictionary<Vector3D, List<IMyOxygenFarm>>();
  solar_grid_vectors = new Dictionary<IMyCubeGrid, Vector3D>();
  oxygen_grid_vectors = new Dictionary<IMyCubeGrid, Vector3D>();
  grid_to_rotor_vector = new Dictionary<IMyCubeGrid, Vector3D>();
  rotors = new Dictionary<Vector3D, List<IMyMotorBase>>();
  name = "";
  large_grid = true;
 }
 private void add<T>(T b, Dictionary<Vector3D, List<T>> blocks,
                     Dictionary<IMyCubeGrid, Vector3D> grid_vectors)
                     where T : IMyTerminalBlock {
  bool valid = false;
  bool needs_grid = true;
  var dir = Vector3D.Abs(b.WorldMatrix.Forward);
  var grid = b.CubeGrid;
  if (grid_vectors.ContainsKey(grid)) {
   var other = grid_vectors[grid];
   if (other == dir) {
    valid = true;
    needs_grid = false;
   } else {
    throw new Exception(b.CustomName + " is misaligned");
   }
  } else {
   valid = true;
  }
  if (valid) {
   if (!grid_to_rotor_vector.ContainsKey(grid)) {
    throw new Exception(b.CustomName + " is not connected to a rotor");
   }
   var grid_vec = grid_to_rotor_vector[grid];
   if (blocks.ContainsKey(grid_vec)) {
    blocks[grid_vec].Add(b);
   } else {
    var list = new List<T>() {b};
    blocks.Add(grid_vec, list);
   }
  }
  if (needs_grid) {
   grid_vectors.Add(grid, dir);
  }
 }
 public void add(IMySolarPanel sp) {
  add<IMySolarPanel>(sp, solar_panels, solar_grid_vectors);
 }
 public void add(IMyOxygenFarm of) {
  add<IMyOxygenFarm>(of, oxygen_farms, oxygen_grid_vectors);
 }
 // adding rotors is an exception
 public void add(IMyMotorBase mb, IMyCubeGrid connected_grid) {
  var dir = Vector3D.Abs(mb.WorldMatrix.Up);
  if (rotors.ContainsKey(dir)) {
   rotors[dir].Add(mb);
  } else if (rotors.Keys.Count == 2) {
   throw new Exception(mb.CustomName + " is misaligned");
  } else {
   var list = new List<IMyMotorBase>() {mb};
   rotors.Add(dir, list);
  }
  grid_to_rotor_vector.Add(connected_grid, dir);
  large_grid = mb.BlockDefinition.ToString().Contains("Large");
 }
 public bool hasSolarPanels() {
  foreach (var pair in solar_panels) {
   foreach (var panel in pair.Value) {
    return true;
   }
  }
  return false;
 }
 public bool hasOxygenFarms() {
  foreach (var pair in oxygen_farms) {
   foreach (var farm in pair.Value) {
    return true;
   }
  }
  return false;
 }
 public float getSolarEfficiency() {
  float cur = 0;
  float total = 0;
  foreach (var pair in solar_panels) {
   foreach (var panel in pair.Value) {
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
  foreach (var pair in oxygen_farms) {
   foreach (var farm in pair.Value) {
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

STASYS_Group parseBlockGroup(List<IMyTerminalBlock> blocks) {
 // we have to get all rotors first
 var g = new STASYS_Group();
 for (int i = blocks.Count - 1; i >= 0; i--) {
  var block = blocks[i];
  showOnHud(block);
  if (block is IMyMotorBase && !(block is IMyMotorSuspension)) {
   var rotor = block as IMyMotorBase;
   var grid = getConnectedGrid(rotor, getLocalGrids());
   if (grid == null) {
    throw new Exception(rotor.CustomName + " is not connected to anything");
   }
   g.add(rotor, grid);
   blocks.RemoveAt(i);
   hideFromHud(block);
  }
 }
 foreach (var block in blocks) {
  showOnHud(block);
  if (block is IMyOxygenFarm) {
   g.add(block as IMyOxygenFarm);
  } else if (block is IMySolarPanel) {
   g.add(block as IMySolarPanel);
  } else {
   throw new Exception("Unexpected block: " + block.CustomName);
  }
  hideFromHud(block);
 }
 return g;
}

/*
 * States
 */

void s_displayStats() {
 if (local_text_panels.Count == 0) {
  return;
 }
 var sb = new StringBuilder();
 float o_e = 0f, s_e = 0f;
 int o_n = 0, s_n = 0;
 foreach (var g in stasys_groups) {
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
  stasys_groups = new List<STASYS_Group>();
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
    var g = parseBlockGroup(blocks);
    g.name = group.Name;
    stasys_groups.Add(g);
   } catch (Exception e) {
    Echo(e.Message);
   }
  }
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
  s_displayStats,
 };
 Me.SetCustomName("STASYS CPU");
 hideFromHud(Me);
 current_state = 0;
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
