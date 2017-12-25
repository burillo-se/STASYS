/*
* STASYS v0.5
*
* (Solar Tracking Alignment SYStem)
*
* Published under "do whatever you want with it" license (aka public domain).
*
*/

const string VERSION = "0.5";
public List<IMyCubeGrid> local_grids = new List<IMyCubeGrid>();
public List<IMyTerminalBlock> local_text_panels = new List<IMyTerminalBlock>();
public STASYS_Control control = null;
public TimeSpan cur_time;

// state machine
Action[] states = null;

int current_state;

// tuples are banned, so...
public struct TwoFloats
{
    public float first;
    public float second;
}

public class SunVectorData
{
    public float efficiency;
    public Vector3D vector;
    public TimeSpan ts;
    public SunVectorData()
    {
        efficiency = 0;
        vector = new Vector3D();
        ts = new TimeSpan();
    }
    public SunVectorData(float eff_in, Vector3D v_in, TimeSpan ts)
    {
        efficiency = eff_in;
        vector = v_in;
        this.ts = ts;
    }
    public bool IsValid()
    {
        return efficiency > 0;
    }
}
// for sorted set
public class SVComparer : IComparer<SunVectorData>
{
    public int Compare(SunVectorData l, SunVectorData r)
    {
        if (l.efficiency < r.efficiency)
        {
            return -1;
        }
        else if (l.efficiency > r.efficiency)
        {
            return 1;
        }
        return 0;
    }
}

public class STASYS_Control
{
    Action<string> MyEcho;
    public enum State
    {
        Init,
        Search,
        Follow,
        Measure,
        Wait,
        Sleep
    }
    float _sun_rotation_rate; // degrees per second
    public float SunRotationRate
    {
        get { return _sun_rotation_rate; }
        private set { _sun_rotation_rate = value; }
    }
    SortedSet<SunVectorData> vectors;
    SunVectorData cv_data;
    Vector3D sun_normal;
    State _cur_state;
    State cur_state
    {
        get { return _cur_state; }
        set
        {
            init_timestamp();
            Echo("entering mode " + value.ToString());
            switch (value)
            {
                case State.Search:
                    manager.search();
                    break;
                case State.Follow:
                    manager.align(cv_data.vector);
                    break;
                case State.Measure:
                    manager.search();
                    break;
                default:
                    // do nothing
                    break;
            }
            _cur_state = value;
        } // set timestamp every time we change state
    }
    Action<string> broadcastData;
    Func<TimeSpan> getCurrentTime;
    const int API_VERSION = 1;
    TimeSpan timestamp;
    STASYS_Group_Manager manager;

    public STASYS_Control(Action<string> bc_func, Func<TimeSpan> ts_func, Action<string> echofunc)
    {
        vectors = new SortedSet<SunVectorData>(new SVComparer());
        cv_data = new SunVectorData();
        sun_normal = new Vector3D();
        SunRotationRate = 0f;
        timestamp = new TimeSpan();
        broadcastData = bc_func;
        getCurrentTime = ts_func;
        MyEcho = echofunc;
        manager = new STASYS_Group_Manager(MyEcho);
        cur_state = State.Init;
    }

    public void processGroup(List<IMyTerminalBlock> blocks, string name)
    {
        if (manager.groupExists(name))
        {
            manager.updateGroup(blocks, name);
        }
        else
        {
            manager.createGroup(blocks, name);
        }
    }

    public int groupCount()
    {
        return manager.groupCount();
    }

    public TwoFloats getEfficiencies()
    {
        return manager.getEfficiencies();
    }

    public TwoFloats getOutputs()
    {
        return manager.getOutputs();
    }

    public bool TryUpdateVectors(SunVectorData v)
    {
        if (vectors.Count < 2)
        {
            vectors.Add(v);
            return true;
        }
        var min = vectors.Min;
        if (min.efficiency < v.efficiency)
        {
            Echo(String.Format("removing vector {0}", min.ToString()));
            Echo(String.Format("adding vector {0}", v.ToString()));
            vectors.Remove(min);
            vectors.Add(v);
            /*
            var v_min = vectors.Min.ts < vectors.Max.ts ? vectors.Min.vector : vectors.Max.vector;
            var v_max = vectors.Min.ts > vectors.Max.ts ? vectors.Min.vector : vectors.Max.vector;
            UpdateSunNormal(v_min, v_max);
            */
            return true;
        }
        return false;
    }

    void Echo(string str)
    {
        MyEcho("Control: " + str);
    }

    void UpdateSunNormal(Vector3D v1, Vector3D v2)
    {
        sun_normal = Vector3D.Normalize(Vector3D.Cross(v1, v2));
    }

    SunVectorData getVectorData()
    {
        return new SunVectorData(manager.getEfficiency(), manager.getVector(), getCurrentTime());
    }

    void init_timestamp()
    {
        timestamp = getCurrentTime();
    }

    public string stateString()
    {
        switch (cur_state)
        {
            case State.Init:
                return "Initializing";
            case State.Search:
                return "Searching";
            case State.Follow:
                return "Following";
            case State.Measure:
                return "Measuring sun rotation plane";
            case State.Wait:
                return "Measuring sun rotation rate";
            case State.Sleep:
                return "No sun found, sleeping";
        }
        return "Invalid";
    }

    // no statics, so code reuse ftw!
    float getAngle(Vector3D normal, Vector3D src, Vector3D dst)
    {
        var src_dot = Vector3D.Dot(src, normal);
        var dst_dot = Vector3D.Dot(dst, normal);
        var src_dst_dot = Vector3D.Dot(src, dst);

        // they may be parallel
        if (Math.Abs(src_dst_dot) > 0.99)
        {
            return 0f;
        }
        var src_proj = Vector3D.Normalize(src - src_dot * normal);
        var dst_proj = Vector3D.Normalize(dst - dst_dot * normal);

        var dot = Vector3D.Dot(src_proj, dst_proj);
        var cross = Vector3D.Normalize(Vector3D.Cross(src_proj, dst_proj));
        var cross_dot = Vector3D.Dot(normal, cross);
        var rad = (float)Math.Acos(dot);
        var deg = (float)Math.Round(MathHelper.ToDegrees(rad), 2);
        var result = dot < 0 ? -(180 - deg) : deg;
        result = cross_dot > 0 ? result : -result;
        return result;
    }

    public void ProcessState()
    {
        bool loop;
        do
        {
            loop = false;
            Echo("current state: " + cur_state.ToString());
            switch (cur_state)
            {
                case State.Init:
                    {
                        if (cv_data.IsValid())
                        {
                            cur_state = State.Follow;
                            loop = true;
                        }
                        else
                        {
                            cur_state = State.Search;
                            loop = true;
                        }
                    }
                    break;
                case State.Search:
                    {
                        if (manager.IsAligned())
                        {
                            // all groups finished searching for sun
                            cv_data = getVectorData();

                            Echo("aligned, vector " + cv_data.vector.ToString());

                            // if efficiency is very bad, go to sleep
                            if (cv_data.efficiency < 0.3)
                            {
                                cur_state = State.Sleep;
                            }
                            else
                            {
                                // otherwise, try updating our sun vectors
                                TryUpdateVectors(cv_data);
                                // broadcast
                                cur_state = State.Follow;
                            }
                        }
                    }
                    break;
                case State.Follow:
                    {
                        // if no sun normal, search for sun
                        if (SunRotationRate == 0f)
                        {
                            if (manager.IsAligned())
                            {
                                Echo("aligned, " + cv_data.vector.ToString());
                                if (cv_data.efficiency >= 0.999)
                                {
                                    cur_state = State.Wait;
                                    loop = true;
                                }
                                else
                                {
                                    cur_state = State.Search;
                                    loop = true;
                                }
                            }
                        }
                        // if both sun plane and rotation rate are known
                        else
                        {
                            // wait until rotation of 2 degrees can be achieved
                            var limit = 2 / SunRotationRate;
                            var cur_ts = getCurrentTime();
                            var seconds = (cur_ts - timestamp).TotalSeconds;
                            if (seconds >= limit && manager.IsAligned())
                            {
                                // rotate to new angle and keep following
                                var deg = SunRotationRate * seconds;
                                var qt = Quaternion.CreateFromAxisAngle(sun_normal, MathHelper.ToRadians((float)deg));

                                // rotate vectors on rotor plane
                                cv_data.vector = Vector3D.Transform(cv_data.vector, qt);

                                cur_state = State.Follow;
                                loop = true;
                            }
                            else if (manager.IsAligned())
                            {
                                // if we are off by more than 10 degrees
                                var cur_angle = (float)Math.Acos(manager.getEfficiency());
                                var prev_angle = (float)Math.Acos(cv_data.efficiency);
                                var diff = Math.Abs(MathHelper.ToDegrees(prev_angle - cur_angle));
                                if (diff >= 10)
                                {
                                    cur_state = State.Search;
                                    loop = true;
                                }
                            }
                        }
                    }
                    break;
                case State.Wait:
                    {
                        // update current vector if efficiency is bigger than what we registered before
                        var vd = getVectorData();
                        if (vd.efficiency >= cv_data.efficiency)
                        {
                            Echo("updating vector, " + vd.vector.ToString());
                            cv_data = vd;
                            cur_state = State.Wait;
                            break;
                        }
                        var cur = vd.efficiency;
                        var prev = cv_data.efficiency;
                        var cur_angle = (float)Math.Acos(cur);
                        var prev_angle = (float)Math.Acos(prev);
                        var diff = Math.Abs(MathHelper.ToDegrees(prev_angle - cur_angle));

                        // difference needs to be enough to get 10 degrees of rotation
                        if (diff > 10)
                        {
                            var time = (getCurrentTime() - timestamp).TotalSeconds;
                            SunRotationRate = (float)(diff / time);
                            Echo("sun rotation speed: " + SunRotationRate.ToString());

                            manager.search();
                            cur_state = State.Measure;
                            loop = true;
                        }
                    }
                    break;
                case State.Measure:
                    {
                        if (manager.IsAligned())
                        {
                            var vd = getVectorData();

                            if (vd.efficiency >= 0.999)
                            {
                                UpdateSunNormal(cv_data.vector, vd.vector);

                                cv_data = vd;

                                TryUpdateVectors(cv_data);
                                cur_state = State.Follow;
                            }
                            else if (vd.efficiency < 0.3)
                            {
                                cur_state = State.Sleep;
                            }
                            else
                            {
                                cur_state = State.Search;
                                loop = true;
                            }
                        }
                    }
                    break;
                case State.Sleep:
                    {
                        // just wait 120 seconds and go for another search
                        var cur_ts = getCurrentTime();
                        var seconds = (cur_ts - timestamp).TotalSeconds;
                        if (seconds >= 120)
                        {
                            cur_state = State.Search;
                            loop = true;
                        }
                    }
                    break;
            }
        } while (loop);
        manager.processState();
    }
}

public class STASYS_Group_Manager
{
    Dictionary<string, STASYS_Group> groups;
    Action<string> MyEcho;

    public STASYS_Group_Manager(Action<string> echofunc)
    {
        groups = new Dictionary<string, STASYS_Group>();
        MyEcho = echofunc;
    }

    public int groupCount()
    {
        return groups.Count;
    }

    public bool groupExists(string name)
    {
        return groups.ContainsKey(name);
    }

    public void createGroup(List<IMyTerminalBlock> blocks, string name)
    {
        var sg = new STASYS_Group(name);

        Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> panels;
        Dictionary<IMyCubeGrid, IMyMotorBase> grid_to_rotor;

        processBlocks(blocks, out panels, out grid_to_rotor);

        sg.update(panels, grid_to_rotor, MyEcho);

        groups.Add(name, sg);
    }

    public void updateGroup(List<IMyTerminalBlock> blocks, string name)
    {
        STASYS_Group sg = groups[name];

        // update existing group only if it's idle
        if (!sg.isFinished())
        {
            return;
        }

        Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> panels;
        Dictionary<IMyCubeGrid, IMyMotorBase> grid_to_rotor;

        processBlocks(blocks, out panels, out grid_to_rotor);

        sg.update(panels, grid_to_rotor, MyEcho);
    }

    public void removeGroup(string name)
    {
        groups.Remove(name);
    }

    void processBlocks(List<IMyTerminalBlock> src_blocks,
        out Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> panels,
        out Dictionary<IMyCubeGrid, IMyMotorBase> grid_to_rotor)
    {
        panels = new Dictionary<IMyCubeGrid, List<IMyTerminalBlock>>();
        grid_to_rotor = new Dictionary<IMyCubeGrid, IMyMotorBase>();

        // pass 1: collect all grids and rotors
        foreach (var b in src_blocks)
        {
            if (b is IMyMotorBase)
            {
                var rotor = b as IMyMotorBase;
                grid_to_rotor.Add(rotor.TopGrid, rotor);
            }
            else if (b is IMyOxygenFarm || b is IMySolarPanel)
            {
                List<IMyTerminalBlock> list;
                if (panels.TryGetValue(b.CubeGrid, out list))
                {
                    list.Add(b);
                }
                else
                {
                    list = new List<IMyTerminalBlock>() { b };
                    panels.Add(b.CubeGrid, list);
                }
            }
        }

        if (panels.Count == 0)
        {
            throw new Exception("No solar panels or oxygen farms found");
        }

        // pass 2: check if there is more than one base rotor
        int n_base = 0;
        IMyMotorBase base_rotor = null;
        var to_remove = new List<IMyCubeGrid>();
        foreach (var p in grid_to_rotor)
        {
            if (!grid_to_rotor.ContainsKey(p.Value.CubeGrid))
            {
                base_rotor = p.Value;
                n_base++;
            }
            // also, find all grids that don't have solar panels or oxygen farms
            if (!panels.ContainsKey(p.Key) && !grid_to_rotor.ContainsKey(p.Value.TopGrid))
            {
                to_remove.Add(p.Key);
            }
        }
        if (n_base != 1)
        {
            throw new Exception("Expected 1 base rotor, found: " + n_base.ToString());
        }

        // remove all extraneous grids
        foreach (var g in to_remove)
        {
            grid_to_rotor.Remove(g);
        }

        if (grid_to_rotor.Count == 0)
        {
            throw new Exception("No solar panels or oxygen farms found");
        }

        // pass 3: check if all blocks are pointing in correct directions
        // that means:
        // 1) all non-base rotors are orthogonal to base rotor
        // 2) all non-base rotors are not orthogonal to each other
        // 3) all solar panels within a grid point in the same direction
        // 4) all solar panels belonging to non-base rotors align on the same plane

        var base_vector = base_rotor.WorldMatrix.Up;
        var aux_vector = new Vector3D();

        // do 1) and 2)
        foreach (var p in grid_to_rotor)
        {
            // skip base rotor
            if (p.Value == base_rotor)
            {
                continue;
            }
            if (Vector3D.IsZero(aux_vector))
            {
                aux_vector = p.Value.WorldMatrix.Up;
                var dot = Math.Abs(Vector3D.Dot(aux_vector, base_vector));
                if (dot > 0.1)
                {
                    p.Value.ShowOnHUD = true;
                    throw new Exception("Rotor " + p.Value.CustomName + " is misaligned");
                }
            }
            else
            {
                var cur_vector = p.Value.WorldMatrix.Up;
                var dot = Math.Abs(Vector3D.Dot(aux_vector, cur_vector));
                if (dot < 0.9)
                {
                    p.Value.ShowOnHUD = true;
                    throw new Exception("Rotor " + p.Value.CustomName + " is misaligned");
                }
            }
        }

        // do 3) and 4)
        foreach (var p in panels)
        {
            // vector to compare against
            var cmp_vector = Vector3D.IsZero(aux_vector) ? base_vector : aux_vector;
            var cur_vector = new Vector3D();
            foreach (var b in p.Value)
            {
                if (Vector3D.IsZero(cur_vector))
                {
                    cur_vector = b.WorldMatrix.Forward;

                    var dot = Math.Abs(Vector3D.Dot(cur_vector, cmp_vector));
                    if (dot > 0.1)
                    {
                        b.ShowOnHUD = true;
                        throw new Exception("Block " + b.CustomName + " is misaligned");
                    }
                }
                else
                {
                    var dot = Math.Abs(Vector3D.Dot(cur_vector, b.WorldMatrix.Forward));
                    if (dot < 0.9)
                    {
                        b.ShowOnHUD = true;
                        throw new Exception(String.Format("Block {0} is misaligned", b.CustomName));
                    }
                }
            }

            // piggyback on this check: make sure all grids are attached to rotors
            if (!grid_to_rotor.ContainsKey(p.Key))
            {
                foreach (var b in p.Value)
                {
                    b.ShowOnHUD = true;
                }
                throw new Exception("Grid not attached to rotor");
            }

            // if this is a grid attached to base rotor, it also must be orthogonal to base vector
            if (grid_to_rotor[p.Key] == base_rotor)
            {
                var dot = Math.Abs(Vector3D.Dot(cur_vector, base_vector));
                if (dot > 0.1)
                {
                    throw new Exception("Grid is misaligned");
                }
            }

            // finally, there may exist panels that are on the same grid as base rotor
            if (p.Key == base_rotor.CubeGrid)
            {
                throw new Exception("Blocks on same grid as base rotor found");
            }
        }
    }

    public void processState()
    {
        foreach (var g in groups)
        {
            g.Value.processState();
        }
    }

    public bool IsAligned()
    {
        foreach (var g in groups)
        {
            if (!g.Value.isFinished())
            {
                return false;
            }
        }
        return true;
    }

    public float getEfficiency()
    {
        float max = 0;
        foreach (var g in groups)
        {
            var e = g.Value.getEfficiency();
            if (e > max)
            {
                max = e;
            }
        }
        return max;
    }

    public TwoFloats getEfficiencies()
    {
        var result = new TwoFloats();
        foreach (var p in groups)
        {
            var g = p.Value;
            var tmp = g.getEfficiencies();
            result.first += tmp.first;
            result.second += tmp.second;
        }
        result.first /= groups.Count;
        result.second /= groups.Count;
        return result;
    }

    public TwoFloats getOutputs()
    {
        var result = new TwoFloats();
        foreach (var p in groups)
        {
            var g = p.Value;
            var tmp = g.getOutputs();
            result.first += tmp.first;
            result.second += tmp.second;
        }
        return result;
    }

    public Vector3D getVector()
    {
        float max_e = 0;
        var v = new Vector3D();
        foreach (var g in groups)
        {
            var e = g.Value.getEfficiency();
            if (Vector3D.IsZero(v) || e > max_e)
            {
                max_e = e;
                v = g.Value.getVector();
            }
        }
        return v;
    }

    public void align(Vector3D v)
    {
        foreach (var g in groups)
        {
            g.Value.align(v);
        }
    }

    public void search()
    {
        foreach (var g in groups)
        {
            g.Value.search();
        }
    }
}

// this class encapsulates a single group, can have multiple axis
public class STASYS_Group
{
    public string name;
    Action<string> MyEcho = null;
    public enum SearchType
    {
        Coarse,
        Fine
    }
    public enum State
    {
        Idle,
        Search,
        Move
    }
    Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> panels;
    Dictionary<IMyCubeGrid, IMyMotorBase> grid_to_rotor;
    bool large_grid;
    SearchType search_type;
    State cur_state;
    List<STASYS_Axis> axis;

    public STASYS_Group(string name)
    {
        this.name = name;
        panels = new Dictionary<IMyCubeGrid, List<IMyTerminalBlock>>();
        grid_to_rotor = new Dictionary<IMyCubeGrid, IMyMotorBase>();
        cur_state = State.Idle;
        search_type = SearchType.Coarse;
        axis = new List<STASYS_Axis>();
    }

    public void reset()
    {
        foreach (var a in axis)
        {
            a.reset();
        }
        cur_state = State.Move;
    }

    void Echo(string str)
    {
        if (MyEcho == null)
        {
            return;
        }
        MyEcho(String.Format("Group {0}: {1}", name, str));
    }

    public void update(Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> new_panels,
        Dictionary<IMyCubeGrid, IMyMotorBase> new_grid_to_rotor, Action<string> echofunc)
    {
        axis.Clear();
        panels = new_panels;
        grid_to_rotor = new_grid_to_rotor;
        this.MyEcho = echofunc;

        // get our base rotor
        IMyMotorBase base_rotor = null;
        foreach (var p in grid_to_rotor)
        {
            var r = p.Value;
            if (!grid_to_rotor.ContainsKey(r.CubeGrid))
            {
                base_rotor = r;
                break;
            }
        }

        // step 1: assign all grids to base rotor
        if (base_rotor != null)
        {
            var lateral_axis_panels = new Dictionary<IMyCubeGrid, List<IMyTerminalBlock>>();
            var lateral_grid_to_rotor = new Dictionary<IMyCubeGrid, IMyMotorBase>();

            foreach (var p in panels)
            {
                lateral_axis_panels.Add(p.Key, p.Value);
                lateral_grid_to_rotor.Add(p.Key, base_rotor);
            }

            var lateral_axis = new STASYS_Axis(lateral_axis_panels, lateral_grid_to_rotor,
                base_rotor.WorldMatrix.Left, "base", Echo, true);
            axis.Add(lateral_axis);
        }

        // step 2: assign all grids to their respective rotors, excluding base rotor grids
        var vertical_axis_panels = new Dictionary<IMyCubeGrid, List<IMyTerminalBlock>>();
        var vertical_grid_to_rotor = new Dictionary<IMyCubeGrid, IMyMotorBase>();

        foreach (var p in panels)
        {
            if (p.Key == base_rotor.TopGrid)
            {
                continue;
            }
            vertical_axis_panels.Add(p.Key, p.Value);
            vertical_grid_to_rotor.Add(p.Key, grid_to_rotor[p.Key]);
        }

        var vertical_axis = new STASYS_Axis(vertical_axis_panels, vertical_grid_to_rotor,
            base_rotor.WorldMatrix.Up, "secondary", Echo);
        axis.Add(vertical_axis);

        var rotor = grid_to_rotor.Values.First();
        large_grid = rotor.BlockDefinition.ToString().Contains("Large");
    }

    public void processState()
    {
        foreach (var a in axis)
        {
            a.processState();
        }
        bool loop;
        do
        {
            Echo("current state: " + cur_state.ToString());
            loop = false;
            switch (cur_state)
            {
                case State.Idle:
                    break;
                case State.Move:
                    foreach (var a in axis)
                    {
                        if (!a.finishedMove())
                        {
                            return;
                        }
                    }
                    foreach (var a in axis)
                    {
                        a.setIdle();
                    }
                    Echo("finished move");
                    cur_state = State.Idle;
                    break;
                case State.Search:
                    var base_axis = axis.First();
                    var secondary_axis = axis.Last();
                    bool has_two_axis = base_axis != secondary_axis;

                    // it may be more efficient to piggy back on multiple
                    // checks at once, but it's far cleaner to just have split
                    // code paths for two cases
                    if (!has_two_axis)
                    {
                        if (base_axis.isReadyToSearch())
                        {
                            base_axis.startSearch();
                        }
                        if (base_axis.finishedMove())
                        {
                            var eff = base_axis.getEfficiency();
                            if (search_type == SearchType.Fine)
                            {
                                base_axis.setIdle();
                                cur_state = State.Idle;
                            }
                            else
                            {
                                // prepare to fine search with base axis
                                search_type = SearchType.Fine;
                                base_axis.prepareToSearch(search_type);
                            }
                        }
                    }
                    else
                    {
                        Echo("doing search");
                        if (base_axis.isReadyToSearch() && secondary_axis.isIdle())
                        {
                            Echo("starting search on base axis");
                            base_axis.startSearch();
                        }
                        if (base_axis.finishedSearch())
                        {
                            Echo("finished search on base axis");
                            // now that the first axis is finished,
                            // dispense with the second axis
                            if (secondary_axis.isIdle())
                            {
                                Echo("preparing search on secondary axis");
                                var eff = base_axis.getLastSearchEfficiency();
                                secondary_axis.prepareToSearch(search_type, eff);
                            }
                            if (base_axis.finishedMove() && secondary_axis.isReadyToSearch())
                            {
                                Echo("starting search on secondary axis");
                                secondary_axis.startSearch();
                            }
                            if (base_axis.finishedMove() && secondary_axis.finishedMove())
                            {
                                var eff = base_axis.getEfficiency();
                                if (search_type == SearchType.Fine)
                                {
                                    Echo("finished fine search");
                                    base_axis.setIdle();
                                    secondary_axis.setIdle();
                                    cur_state = State.Idle;
                                }
                                else
                                {
                                    // prepare to fine search with base axis
                                    search_type = SearchType.Fine;
                                    Echo("starting fine search");
                                    var last_eff = secondary_axis.getLastSearchEfficiency();
                                    base_axis.prepareToSearch(search_type, last_eff);
                                    secondary_axis.setIdle();
                                }
                            }
                        }
                    }
                    break;
            }
        } while (loop);
    }

    public void search()
    {
        float max = 0;
        bool has_single_axis = axis.Count == 1;
        foreach (var a in axis)
        {
            var e = a.getEfficiency();
            if (e > max)
            {
                max = e;
            }
        }
        if (has_single_axis)
        {
            if (max >= 0.8)
            {
                search_type = SearchType.Fine;
            }
            else
            {
                search_type = SearchType.Coarse;
            }
            axis.First().prepareToSearch(search_type, max);
        }
        else
        {
            Echo("preparing for search");
            bool misaligned = axis.Last().isMisaligned();

            if (max >= 0.8 && !misaligned)
            {
                Echo("init fine search");
                search_type = SearchType.Fine;
            }
            else
            {
                Echo("init coarse search");
                search_type = SearchType.Coarse;
                if (max <= 0.2 || misaligned)
                    axis.Last().reset();
            }
            axis.First().prepareToSearch(search_type, max);
        }
        cur_state = State.Search;
    }

    public void align(Vector3D v)
    {
        foreach (var a in axis)
        {
            a.alignToVector(v);
        }
        cur_state = State.Move;
    }

    public bool isFinished()
    {
        return cur_state == State.Idle;
    }

    public Vector3D getVector()
    {
        var res = axis.Last().getVector();
        Echo("get vector: " + res.ToString());
        return res;
    }

    public float getEfficiency()
    {
        var e = axis.First().getEfficiency();
        if (axis.Count > 1)
        {
            var s_e = axis.Last().getEfficiency();
            if (s_e > e)
            {
                e = s_e;
            }
        }
        return e;
    }

    public TwoFloats getEfficiencies()
    {
        float sol_cur = 0;
        float sol_total = 0;
        float oxy_cur = 0;
        float oxy_total = 0;
        foreach (var pair in panels)
        {
            foreach (var block in pair.Value)
            {
                if (block is IMySolarPanel)
                {
                    var panel = block as IMySolarPanel;
                    float max = large_grid ? 0.120f : 0.30f;
                    sol_total += max;
                    sol_cur += panel.MaxOutput;
                }
                else if (block is IMyOxygenFarm)
                {
                    var farm = block as IMyOxygenFarm;
                    float max = 1f;
                    oxy_total += max;
                    oxy_cur += farm.GetOutput();
                }
            }
        }
        var sol_rate = sol_total == 0 ? 0 : sol_cur / sol_total;
        var oxy_rate = oxy_total == 0 ? 0 : oxy_cur / oxy_total;
        var res = new TwoFloats();
        res.first = sol_rate;
        res.second = oxy_rate;
        return res;
    }

    public TwoFloats getOutputs()
    {
        float solar_output = 0;
        float oxygen_output = 0;
        foreach (var pair in panels)
        {
            foreach (var block in pair.Value)
            {
                if (block is IMySolarPanel)
                {
                    var panel = block as IMySolarPanel;
                    solar_output += panel.MaxOutput;
                }
                else if (block is IMyOxygenFarm)
                {
                    var farm = block as IMyOxygenFarm;
                    oxygen_output += farm.GetOutput() * 1.8f;
                }
            }
        }
        // solar output in kiloWatts, oxygen output in liters
        var res = new TwoFloats();
        res.first = solar_output * 1000f;
        res.second = oxygen_output;
        return res;
    }

}

// this is a class that uses a collection of rotors and
// solar/oxygen blocks to find sun (or rotate towards it). if there
// are many rotors, they are all aligned on the same axis, and are
// moving with the same speed and in the same direction always.
public class STASYS_Axis
{
    Action<string> MyEcho;
    enum State
    {
        Idle,
        Prepare,
        Search,
        Move,
        Finished,
        Override
    }
    State _cur_state;
    State cur_state
    {
        get { return _cur_state; }
        set
        {
            Echo("changing state to " + value.ToString());
            switch (value)
            {
                case State.Finished:
                case State.Idle:
                    center_vector = new Vector3D();
                    search_data.Clear();
                    foreach (var p in grid_to_rotor)
                    {
                        var r = p.Value;
                        stopRotor(r);
                    }
                    break;
                case State.Prepare:
                    search_data.Clear();
                    calcMinMaxVectors();
                    Echo("aligning to vector " + min_vector.ToString());
                    align(min_vector, true);
                    fixTargets();
                    break;
                case State.Search:
                    if (!base_axis)
                    {
                        Echo("transforming max vector back to world coords");
                        var matrix = MatrixD.Transpose(grid_to_rotor.First().Value.CubeGrid.WorldMatrix).GetOrientation();
                        max_vector = Vector3D.Transform(max_vector, matrix);
                    }
                    Echo("aligning to vector " + max_vector.ToString());
                    align(max_vector, cur_type == STASYS_Group.SearchType.Coarse, cur_type == STASYS_Group.SearchType.Coarse);
                    break;
                case State.Move:
                case State.Override:
                    Echo("aligning to vector " + target.ToString());
                    align(target, true, false, value == State.Override); // override means fuck alignment
                    fixTargets();
                    break;

            }
            pending_search_start_efficiency = 0;
            _cur_state = value;
        }
    }
    STASYS_Group.SearchType cur_type;
    struct SearchData
    {
        public SearchData(Vector3D v_in, float e_in)
        {
            v = v_in;
            val = e_in;
        }
        public bool isValid()
        {
            return !Vector3D.IsZero(v);
        }
        public Vector3D v;
        public float val;
    }
    struct SearchDataResult
    {
        public SearchData data;
        public int idx;
    }
    List<SearchData> search_data;
    Vector3D min_vector, max_vector;
    Vector3D center_vector;
    float pending_search_start_efficiency;
    float last_max_efficiency;
    Vector3D target;
    Vector3D base_normal;
    bool large_grid;
    bool base_axis;
    Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> panels;
    Dictionary<IMyCubeGrid, IMyMotorBase> grid_to_rotor;
    string name;

    public STASYS_Axis(Dictionary<IMyCubeGrid, List<IMyTerminalBlock>> panels_in,
        Dictionary<IMyCubeGrid, IMyMotorBase> grid_to_rotor_in, Vector3D default_center,
        string name, Action<string> echofunc, bool base_axis = false)
    {
        panels = panels_in;
        grid_to_rotor = grid_to_rotor_in;
        cur_type = STASYS_Group.SearchType.Coarse;
        search_data = new List<SearchData>();
        center_vector = new Vector3D();
        min_vector = new Vector3D();
        max_vector = new Vector3D();
        target = new Vector3D();
        pending_search_start_efficiency = 0;
        last_max_efficiency = 0;
        this.name = name;

        var rotor = grid_to_rotor.Values.First();
        large_grid = rotor.BlockDefinition.ToString().Contains("Large");
        this.base_axis = base_axis;

        base_normal = default_center;
        MyEcho = echofunc;
        _cur_state = State.Idle; // bypass state machine
    }

    void Echo(string str)
    {
        MyEcho(String.Format("Axis {0}: {1}", name, str));
    }

    public bool isMisaligned()
    {
        var vec = new Vector3D();
        var rotor_normal = grid_to_rotor.First().Value.WorldMatrix.Up;
        foreach (var p in grid_to_rotor)
        {
            if (Vector3D.IsZero(vec))
            {
                vec = getGridVector(p.Key);
                continue;
            }
            if (Math.Abs(getAngle(rotor_normal, vec, getGridVector(p.Key))) >= 1)
            {
                return true;
            }
        }
        return false;
    }

    public void reset()
    {
        target = getDefaultCenter();
        cur_state = State.Override;
    }

    public void alignToVector(Vector3D v)
    {
        Echo("asked to align to vector: " + v.ToString());
        target = projectVector(v);
        Echo("projected vector: " + target.ToString());
        cur_state = State.Move;
    }

    public Vector3D getVector()
    {
        var res = getGridVector(grid_to_rotor.First().Key);
        Echo("get vector: " + res.ToString());
        return res;
    }

    public void setIdle()
    {
        cur_state = State.Idle;
    }

    public void prepareToSearch(STASYS_Group.SearchType type, float eff = 0)
    {
        Echo("preparing to search");
        cur_type = type;
        if (getEfficiency() >= 0.999 && type == STASYS_Group.SearchType.Fine)
        {
            cur_state = State.Finished;
        }
        else
        {
            pending_search_start_efficiency = eff;
            cur_state = State.Prepare;
        }
    }

    public bool isReadyToSearch()
    {
        return cur_state == State.Prepare && reachedTargetVector();
    }

    public void startSearch()
    {
        cur_state = State.Search;
    }

    public bool finishedSearch()
    {
        return cur_state == State.Move || cur_state == State.Finished;
    }

    public bool finishedMove()
    {
        return cur_state == State.Finished;
    }

    public bool isIdle()
    {
        return cur_state == State.Idle;
    }

    public float getLastSearchEfficiency()
    {
        return last_max_efficiency;
    }

    public float getEfficiency()
    {
        float max = 0;
        foreach (var pair in panels)
        {
            foreach (var block in pair.Value)
            {
                var cur = 0f;
                var cur_max = 1f;
                if (block is IMySolarPanel)
                {
                    var panel = block as IMySolarPanel;
                    cur_max = large_grid ? 0.120f : 0.30f;
                    cur = panel.MaxOutput;
                }
                else if (block is IMyOxygenFarm)
                {
                    var farm = block as IMyOxygenFarm;
                    cur_max = 1f;
                    cur = farm.GetOutput();
                }
                if (cur / cur_max > max)
                {
                    max = cur / cur_max;
                }
            }
        }
        return (float)Math.Round(max, 4);
    }

    public void processState()
    {
        bool loop;
        do
        {
            Echo("current state: " + cur_state.ToString());
            switch (cur_state)
            {
                default:
                    break;
                case State.Override:
                    if (reachedTargetVector())
                    {
                        cur_state = State.Idle;
                    }
                    break;
                case State.Prepare:
                    // if this isn't a second fine search, bail out
                    if (!Vector3D.IsZero(center_vector))
                    {
                        break;
                    }
                    // if moved to initial position, start search
                    if (reachedTargetVector())
                    {
                        cur_state = State.Search;
                        loop = true;
                    }
                    break;
                case State.Search:
                    var eff = getEfficiency();
                    var vec = getVector();
                    Echo(String.Format("vector {0}: efficiency {1}", vec.ToString(), eff));
                    if (eff >= 0.999 && cur_type == STASYS_Group.SearchType.Fine)
                    {
                        target = vec;
                        cur_state = State.Move;
                        loop = true;
                        break;
                    }
                    search_data.Add(new SearchData(vec, eff));
                    if (reachedTargetVector())
                    {
                        // we reached our search limit, now do some calculations
                        var t = getSearchResults();
                        var idx = t.idx;
                        var data = t.data;
                        Echo(String.Format("max vector {0}: efficiency {1}", data.v.ToString(), data.val));
                        if (data.val < 0.2)
                        {
                            // we didn't find a suitably good max point
                            cur_state = State.Finished;
                            last_max_efficiency = data.val;
                            break;
                        }
                        // find where our maximum was
                        var max = idx == 0 ? data.v :
                            Vector3D.Normalize((data.v + search_data[idx - 1].v) / 2);
                        if (cur_type == STASYS_Group.SearchType.Coarse)
                        {
                            // for coarse searches, just move to maximum
                            target = max;
                            last_max_efficiency = data.val;
                            cur_state = State.Move;
                            break;
                        }
                        var pos = Math.Abs((float)idx / search_data.Count - 0.5);

                        // if our maximum is somewhere in the center, or if this isn't
                        // our first search, proceed
                        if (pos <= 0.15 || !Vector3D.IsZero(center_vector))
                        {
                            target = max;
                            last_max_efficiency = data.val;
                            cur_state = State.Move;
                        }
                        else
                        // if our maximum was somewhere outside center, redo the search
                        {
                            Echo(String.Format("restarting fine search at {0}", max.ToString()));
                            center_vector = max;
                            pending_search_start_efficiency = data.val;
                            cur_state = State.Prepare;
                        }
                    }
                    break;
                case State.Move:
                    if (reachedTargetVector())
                    {
                        cur_state = State.Finished;
                    }
                    break;
            }
            loop = false;
        } while (loop);
    }

    // put vector on our rotor plane
    Vector3D projectVector(Vector3D v)
    {
        // now, put this vector on our rotor plane
        var rotor = grid_to_rotor[panels.First().Key];

        // find rotor plane normal
        var rotor_normal = rotor.WorldMatrix.Up;

        // find dot product between our vector, and rotor plane normal
        var dot = Vector3D.Dot(v, rotor_normal);

        // find projection of our vector onto the plane
        var proj = Vector3D.Normalize(v - dot * rotor_normal);

        Echo(String.Format("projecting {0}: {1}", v.ToString(), proj.ToString()));

        return proj;
    }

    Vector3D getDefaultCenter()
    {
        var rotor = grid_to_rotor.First().Value;
        var r_normal = rotor.WorldMatrix.Up;

        var qt = Quaternion.CreateFromAxisAngle(r_normal, MathHelper.ToRadians(-90));
        var vec = Vector3D.Transform(base_normal, qt);
        return projectVector(Vector3D.Transform(base_normal, qt));
    }

    float getSearchRange(float eff)
    {
        // extend range by 30%
        var angle = (float)MathHelper.ToDegrees((float)Math.Acos(eff)) * 1.3f;
        if (angle > 85)
        {
            angle = 85f;
        }
        return angle;
    }

    float getRotorSpeed(float range)
    {
        var deg_per_s = range / 17; // expect 17 seconds for any range, smaller ranges will be slower
        var deg_per_m = deg_per_s * 60;
        return deg_per_m / 360;
    }

    void calcMinMaxVectors(float eff = 0)
    {
        // if center vector is set, use that as center
        var rotor = grid_to_rotor.First().Value;
        var cur = getVector();
        Vector3D rot_vec;
        if (!Vector3D.IsZero(center_vector))
        {
            rot_vec = center_vector;
            eff = pending_search_start_efficiency;
        }
        else
        {
            if (cur_type == STASYS_Group.SearchType.Coarse)
            {
                rot_vec = getDefaultCenter();
            }
            else
            {
                rot_vec = cur;
                eff = pending_search_start_efficiency;
            }
        }

        // find rotor plane normal
        var rotor_normal = rotor.WorldMatrix.Up;

        // create rotation matrices for that plane normal
        var deg = getSearchRange(eff);
        var pos_qt = Quaternion.CreateFromAxisAngle(rotor_normal, MathHelper.ToRadians(deg));
        var neg_qt = Quaternion.CreateFromAxisAngle(rotor_normal, MathHelper.ToRadians(-deg));

        // rotate vectors on rotor plane
        max_vector = Vector3D.Transform(rot_vec, neg_qt);
        min_vector = Vector3D.Transform(rot_vec, pos_qt);

        // if we're closer to max than min, reverse them
        if (Math.Abs(getAngle(rotor_normal, cur, max_vector)) < Math.Abs(getAngle(rotor_normal, cur, min_vector)))
        {
            var tmp = min_vector;
            min_vector = max_vector;
            max_vector = tmp;
        }
        Echo(String.Format("min {0}: max: {1}", min_vector.ToString(), max_vector.ToString()));

        if (base_axis)
        {
            // nothing further to be done if we're a base axis
            return;
        }

        // now, a magic trick!
        // by the time we are "prepared for search", we have no idea if other axes would rotate us,
        // and that screws up min/max calculation, as we're using global vectors.
        // so what we'll do is, we'll convert our max vector to local coordinates for first rotor
        // (we can't know which one, but we always get the same one so it doesn't matter).
        // then, when the time comes, we will convert that vector to global, thereby avoiding this issue.

        var matrix = rotor.CubeGrid.WorldMatrix.GetOrientation();
        max_vector = Vector3D.Transform(max_vector, matrix);
    }

    // find max item and its index
    SearchDataResult getSearchResults()
    {
        int max_idx = -1;
        var max = new SearchData();
        bool first = true;
        for (int i = 0; i < search_data.Count; i++)
        {
            var d = search_data[i];
            if (first || d.val > max.val)
            {
                max_idx = i;
                max = d;
            }
            first = false;
        }
        var res = new SearchDataResult();
        res.idx = max_idx;
        res.data = max;
        return res;
    }

    float getAngle(Vector3D rotor_normal, Vector3D src, Vector3D dst)
    {
        var src_dot = Vector3D.Dot(src, rotor_normal);
        var dst_dot = Vector3D.Dot(dst, rotor_normal);
        var src_dst_dot = Vector3D.Dot(src, dst);

        // they may be parallel
        if (Math.Abs(src_dst_dot) > 0.99999)
        {
            Echo(String.Format("src: {0} dst: {1} parallel", src.ToString(), dst.ToString()));
            return 0f;
        }
        var src_proj = Vector3D.Normalize(src - src_dot * rotor_normal);
        var dst_proj = Vector3D.Normalize(dst - dst_dot * rotor_normal);

        var dot = Vector3D.Dot(src_proj, dst_proj);
        dot = Math.Max(dot, -1);
        dot = Math.Min(dot, 1);
        var cross = Vector3D.Normalize(Vector3D.Cross(src_proj, dst_proj));
        var cross_dot = Vector3D.Dot(rotor_normal, cross);
        var rad = (float)Math.Acos(dot);
        var deg = (float)Math.Round(MathHelper.ToDegrees(rad), 2);
        var result = deg;
        // if this rotor needs full range (base rotor of a multi-rotor panel), then leave
        // the angle alone, as it's correct; otherwise, truncate it to sharp angle
        if (!base_axis)
        {
            result = dot < 0 ? -(180 - deg) : deg;
        }
        result = cross_dot > 0 ? result : -result;

        Echo(String.Format("angle between {0} and {1}: {2}", src.ToString(), dst.ToString(), result));
        return result;
    }

    Vector3D getGridVector(IMyCubeGrid grid)
    {
        var p_list = panels[grid];
        var p = p_list.First();
        var dir = p.WorldMatrix.Forward;
        var vec = projectVector(dir);
        return vec;
    }

    bool reachedTargetVector()
    {
        foreach (var p in grid_to_rotor)
        {
            var r = p.Value;
            var cur_angle = getRotorAngle(r);
            var u_limit = r.GetValueFloat("UpperLimit");
            var l_limit = r.GetValueFloat("LowerLimit");
            var vel = r.GetValueFloat("Velocity");
            var limit = vel > 0 ? u_limit : l_limit;

            // if velocity is zero, someone stopped us
            if (vel != 0 && (Math.Abs(cur_angle - limit) > 1))
            {
                Echo("not reached target");
                return false;
            }
        }
        Echo("reached target");
        return true;
    }

    // keep target angle within first 180 degrees
    float getTargetAngle(float cur, float offset)
    {
        var res = cur - normalizeAngle(offset);
        Echo(String.Format("cur: {0} offset: {1} target: {2}", cur, offset, res));
        return res;
    }

    void align(Vector3D v, bool fast, bool force_obtuse = false, bool no_invert = false)
    {
        target = v;

        // store rotation plane, so that every rotor rotates in the same direction
        var search_normal = new Vector3D();
        bool? search_dir = null;

        Echo(String.Format("asked to align on: {0}", target.ToString()));

        foreach (var p in panels)
        {
            var r = grid_to_rotor[p.Key];

            var normal = r.WorldMatrix.Up;
            if (Vector3D.IsZero(search_normal))
            {
                search_normal = normal;
            }
            // invert the angle if this rotor has a different normal
            bool inverted = search_normal != normal;

            var offset = getAngle(normal, getGridVector(p.Key), target);
            var cur_angle = getRotorAngle(r);
            var target_angle = getTargetAngle(cur_angle, offset);

            Echo(String.Format("target_angle: {0}", target_angle));

            if (Math.Abs(cur_angle - target_angle) >= 180)
            {
                throw new Exception(String.Format("Difference too big: {0:0} {1:0} {2:0}", cur_angle, target_angle, Math.Abs(cur_angle - target_angle)));
            }

            var dir = cur_angle < target_angle;
            if (search_dir == null)
            {
                search_dir = dir;
            }

            // invert if we're on a different plane and are rotating
            // in the same direction
            bool invert = inverted && search_dir == dir;

            // invert if we're on the same plane and are rotating
            // in different directions
            invert |= !inverted && search_dir != dir;

            if (force_obtuse)
            {
                Echo(String.Format("angle must be obtuse"));
                offset = cur_angle - target_angle;
                bool neg = offset < 0;
                var new_offset = (180 - Math.Abs(offset)) * (neg ? 1 : -1);
                target_angle = cur_angle - new_offset;
                Echo(String.Format("target_angle: {0}", target_angle));

                if (Math.Abs(new_offset) >= 180)
                {
                    throw new Exception(String.Format("Obtuse fail: {0:0} {1:0} {2:0}", cur_angle, target_angle, new_offset));
                }

            }
            if (!no_invert && invert)
            {
                Echo(String.Format("direction must be inverted"));
                offset = cur_angle - target_angle;
                var new_offset = -offset;
                target_angle = cur_angle - new_offset;
                Echo(String.Format("target_angle: {0}", target_angle));

                if (Math.Abs(cur_angle - target_angle) >= 180)
                {
                    throw new Exception(String.Format("Invert fail: {0:0} {1:0} {2:0}", cur_angle, target_angle, Math.Abs(cur_angle - target_angle)));
                }
            }
            moveRotor(r, target_angle, fast);
        }
    }

    // go through each rotor and make sure the end angle is in sane range
    // this prevents panels going overboard with huge current angles
    // we only do this in move state, so there's no interference with
    // searching or otherwise
    void fixTargets()
    {
        foreach (var p in grid_to_rotor)
        {
            var r = p.Value;
            var vel = r.GetValueFloat("Velocity");
            if (vel == 0)
            {
                continue;
            }
            var dir = vel > 0;
            var target = dir ? r.GetValueFloat("UpperLimit") : r.GetValueFloat("LowerLimit");
            var norm_target = (float)Math.Round(normalizeAngle(target, true), 0);
            moveRotor(r, norm_target, true);
        }
    }

    float getRotorAngle(IMyTerminalBlock b)
    {
        var angle_regex = new System.Text.RegularExpressions.Regex("([\\-]?\\d+)");
        var cur_match = angle_regex.Match(b.DetailedInfo);
        if (!cur_match.Success)
        {
            return 0;
        }
        var val = float.Parse(cur_match.Groups[1].Value);
        return val;
    }

    float normalizeAngle(float angle, bool correct = false)
    {
        float limit = correct ?
            base_axis ? 360 : 180
            : 180;
        float half = limit / 2;
        while (angle >= half)
        {
            angle -= limit;
        }
        while (angle < -half)
        {
            angle += limit;
        }
        return angle;
    }

    void moveRotor(IMyMotorBase rotor, float target, bool fast)
    {
        if (target > 360 || target < -360)
        {
            throw new Exception("Unexpectedly big angle, something likely went wrong");
        }
        var cur_angle = getRotorAngle(rotor);
        bool right = target > cur_angle;
        var offset = Math.Abs(cur_angle - target);
        // slow speed is variable
        float speed = fast ? 2f : getRotorSpeed(offset);
        Echo("Rotation speed: " + speed.ToString());
        rotor.SetValue("Velocity", right ? speed : -speed);
        rotor.SetValue("UpperLimit", right ? target : cur_angle);
        rotor.SetValue("LowerLimit", right ? cur_angle : target);
        rotor.SetValue("Weld speed", 20f);
    }

    void stopRotor(IMyMotorBase rotor)
    {
        rotor.SetValue("Velocity", 0f);
        rotor.SetValue("Weld speed", 2f);
    }
}

/*
 * Grid stuff
 */
// grid graph edge class, represents a connection point between two grids.
public class Edge<T>
{
    public T src { get; set; }
    public T dst { get; set; }
}

// comparer for graph edges - the way the comparison is done means the edges are
// bidirectional - meaning, it doesn't matter which grid is source and which
// grid is destination, they will be equal as far as comparison is concerned.
public class EdgeComparer<T> : IEqualityComparer<Edge<T>>
{
    public int GetHashCode(Edge<T> e)
    {
        // multiply src hashcode by dst hashcode - multiplication is commutative, so
        // result will be the same no matter which grid was source or destination
        return e.src.GetHashCode() * e.dst.GetHashCode();
    }
    public bool Equals(Edge<T> e1, Edge<T> e2)
    {
        if (e1.src.Equals(e2.src) && e1.dst.Equals(e2.dst))
        {
            return true;
        }
        if (e1.src.Equals(e2.dst) && e1.dst.Equals(e2.src))
        {
            return true;
        }
        return false;
    }
}

// our grid graph
public class Graph<T>
{
    public Graph()
    {
        cmp = new EdgeComparer<T>();
        v_edges = new Dictionary<T, HashSet<Edge<T>>>();
        r_edges = new HashSet<Edge<T>>(cmp);
    }

    // add an edge to the graph
    public void addEdge(T src, T dst, bool is_remote)
    {
        var t = new Edge<T>();
        t.src = src;
        t.dst = dst;

        // remote edges don't need to be added to local list of edges
        if (is_remote)
        {
            r_edges.Add(t);
            return;
        }

        // add edge to list of per-vertex edges
        HashSet<Edge<T>> hs_src, hs_dst;
        if (!v_edges.TryGetValue(src, out hs_src))
        {
            hs_src = new HashSet<Edge<T>>(cmp);
            v_edges.Add(src, hs_src);
        }
        if (!v_edges.TryGetValue(dst, out hs_dst))
        {
            hs_dst = new HashSet<Edge<T>>(cmp);
            v_edges.Add(dst, hs_dst);
        }
        hs_src.Add(t);
        hs_dst.Add(t);
    }

    // get all grids that are local to source grid (i.e. all grids connected by
    // rotors or pistons)
    public List<T> getGridRegion(T src)
    {
        // if there never was a local edge from/to this grid, it's by definition
        // the only grid in this region
        if (!v_edges.ContainsKey(src))
        {
            return new List<T>() { src };
        }
        // otherwise, gather all vertices in this region
        var region = new List<T>();
        var seen = new HashSet<T>();
        var next = new Queue<T>();
        next.Enqueue(src);
        while (next.Count != 0)
        {
            var g = next.Dequeue();
            if (!seen.Contains(g))
            {
                var edges = v_edges[g];
                foreach (var edge in edges)
                {
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
    public void validateGraph()
    {
        var to_remove = new HashSet<Edge<T>>(cmp);
        var seen = new HashSet<T>();
        foreach (var edge in r_edges)
        {
            var next = new Queue<T>();
            next.Enqueue(edge.src);
            next.Enqueue(edge.dst);
            while (next.Count != 0)
            {
                var g = next.Dequeue();
                if (!seen.Contains(g))
                {
                    var region = new HashSet<T>(getGridRegion(g));
                    seen.UnionWith(region);
                    // find any edges that are completely inside this region, and remove them
                    foreach (var e in r_edges)
                    {
                        if (region.Contains(e.src) && region.Contains(e.dst))
                        {
                            to_remove.Add(e);
                        }
                    }
                }
            }
        }
        foreach (var e in to_remove)
        {
            r_edges.Remove(e);
        }
    }

    // get all neighboring (connected by connectors) grid entry points
    public List<Edge<T>> getGridConnections()
    {
        return new List<Edge<T>>(r_edges);
    }

    // our comparer to use with all sets
    EdgeComparer<T> cmp;
    // list of all edges
    HashSet<Edge<T>> r_edges;
    // dictionaries of edges for each vertex
    Dictionary<T, HashSet<Edge<T>>> v_edges;
}

bool localGridFilter(IMyTerminalBlock b)
{
    return local_grids.Contains(b.CubeGrid);
}

public void filterLocalGrid(List<IMyTerminalBlock> blocks)
{
    for (int i = blocks.Count - 1; i >= 0; i--)
    {
        var b = blocks[i];
        if (!local_grids.Contains(b.CubeGrid))
        {
            blocks.RemoveAt(i);
        }
    }
}

public void filterLocalGrid<T>(List<IMyTerminalBlock> blocks)
{
    for (int i = blocks.Count - 1; i >= 0; i--)
    {
        var b = blocks[i];
        if (!(b is T) || !local_grids.Contains(b.CubeGrid))
        {
            blocks.RemoveAt(i);
        }
    }
}

IMyCubeGrid getConnectedGrid(IMyShipConnector c)
{
    if (c.Status != MyShipConnectorStatus.Connected)
    {
        return null;
    }
    // skip connectors connecting to the same grid
    var o = c.OtherConnector;
    if (o.CubeGrid == c.CubeGrid)
    {
        return null;
    }
    return o.CubeGrid;
}

IMyCubeGrid getConnectedGrid(IMyMotorBase r)
{
    if (!r.IsAttached)
    {
        return null;
    }
    return r.TopGrid;
}

IMyCubeGrid getConnectedGrid(IMyPistonBase p)
{
    if (!p.IsAttached)
    {
        return null;
    }
    return p.TopGrid;
}

// getting local grids is not trivial, we're basically building a graph of all
// grids and figure out which ones are local to us. we are also populating
// object lists in the meantime
void getLocalGrids()
{
    var pistons = new List<IMyTerminalBlock>();
    var rotors = new List<IMyTerminalBlock>();
    var connectors = new List<IMyTerminalBlock>();

    // get all blocks that are accessible to GTS
    var local_blocks = new List<IMyTerminalBlock>();
    GridTerminalSystem.GetBlocks(local_blocks);

    // for each block, get its grid, store data for this grid, and populate respective
    // object list if it's one of the objects we're interested in
    foreach (var b in local_blocks)
    {
        if (b is IMyShipConnector)
        {
            connectors.Add(b);
        }
        else if (b is IMyPistonBase)
        {
            pistons.Add(b);
        }
        else if (b is IMyMotorBase)
        {
            rotors.Add(b);
        }
    }

    // now, build a graph of all grids
    var gr = new Graph<IMyCubeGrid>();

    // first, go through all pistons
    foreach (IMyPistonBase p in pistons)
    {
        var connected_grid = getConnectedGrid(p);

        if (connected_grid != null)
        {
            // grids connected to pistons are local to their source
            gr.addEdge(p.CubeGrid, connected_grid, false);
        }
    }

    // do the same for rotors
    foreach (IMyMotorBase rotor in rotors)
    {
        var connected_grid = getConnectedGrid(rotor);

        if (connected_grid != null)
        {
            // grids connected to locals are local to their source
            gr.addEdge(rotor.CubeGrid, connected_grid, false);
        }
    }

    // do the same for connectors
    foreach (IMyShipConnector c in connectors)
    {
        var connected_grid = getConnectedGrid(c);

        if (connected_grid != null)
        {
            // grids connected to connectors belong to a different ship
            gr.addEdge(c.CubeGrid, connected_grid, true);
        }
    }

    // make sure we remove all unnecessary edges from the graph
    gr.validateGraph();

    // now, get our actual local grid
    local_grids = gr.getGridRegion(Me.CubeGrid);
}

/*
 * Delegates
 */
void echo(string str)
{
    Me.CustomData += str + "\r\n";
}

void broadcast(string val)
{

}

TimeSpan runtime()
{
    return cur_time;
}

/*
 * Misc functions
 */
string getUnitStr(float v)
{
    var pwrs = "kMGTPEZY";
    int pwr_idx = 0;
    while (v >= 1000)
    {
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

string getTimeStr(float sec)
{
    string[] quals = { "secs", "mins", "hrs", "days", "wks" };
    int[] divs = { 60, 60, 24, 7 };

    int i;
    for (i = 0; i < divs.Length; i++)
    {
        int div = divs[i];
        if (sec < div)
        {
            break;
        }
        sec /= div;
    }
    return String.Format("{0:0} {1}", sec, quals[i]);
}

/*
* States
*/
void s_processStates()
{
    control.ProcessState();
}

void s_displayStats()
{
    if (local_text_panels.Count == 0)
    {
        return;
    }
    var sb = new StringBuilder();
    var output = control.getOutputs();
    var efficiencies = control.getEfficiencies();

    sb.AppendLine(String.Format("STASYS v{0}", VERSION));
    sb.AppendLine(String.Format("Groups under control: {0}", control.groupCount()));
    sb.AppendLine(String.Format("Oxygen farm efficiency: {0:0.0}%", efficiencies.second * 100f));
    sb.AppendLine(String.Format("Oxygen farm output: {0:0}L", output.second));
    sb.AppendLine(String.Format("Solar panel efficiency: {0:0.0}%", efficiencies.first * 100f));
    sb.AppendLine(String.Format("Solar panel output: {0}W", getUnitStr(output.first)));
    if (control.SunRotationRate == 0)
    {
        sb.AppendLine("Measured sun rotation rate: Unknown");
    }
    else
    {
        sb.AppendLine(String.Format("Measured sun rotation rate: {0}", getTimeStr(360 / control.SunRotationRate)));
    }
    sb.AppendLine(String.Format("State: {0}", control.stateString()));
    foreach (IMyTextPanel panel in local_text_panels)
    {
        panel.WritePublicTitle("STASYS Notification");
        panel.WritePublicText(sb.ToString());
        panel.ShowPublicTextOnScreen();
    }
}

void s_refreshGroups()
{
    var groups = new List<IMyBlockGroup>();
    GridTerminalSystem.GetBlockGroups(groups);
    for (int i = 0; i < groups.Count; i++)
    {
        var group = groups[i];
        // skip groups we don't want
        if (group.Name == "STASYS Notify")
        {
            var txt = new List<IMyTerminalBlock>();
            group.GetBlocks(txt);
            filterLocalGrid<IMyTextPanel>(txt);
            local_text_panels = txt;
            continue;
        }
        if (!group.Name.StartsWith("STASYS"))
        {
            continue;
        }
        var blocks = new List<IMyTerminalBlock>();
        group.GetBlocks(blocks);
        filterLocalGrid(blocks);
        if (blocks.Count == 0)
        {
            // this group is from a foreign grid
            continue;
        }
        try
        {
            var name = group.Name;
            control.processGroup(blocks, name);
        }
        catch (Exception e)
        {
            Echo(group.Name + ": " + e.Message);
        }
    }
}

void s_refreshGrids()
{
    getLocalGrids();
}

int[] state_cycle_counts;
int cycle_count;

bool canContinue()
{
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
    Decimal cycle_percentage = (Decimal)projected_cycle_count / Runtime.MaxInstructionCount;

    // to speed up initial run, keep 40% headroom for next states
    bool initRunCycleHeadroom = isFirstRun && cycle_percentage <= 0.4M;

    bool runCycleHeadroom = !isFirstRun && cycle_percentage <= 0.8M;

    if (initRunCycleHeadroom || runCycleHeadroom)
    {
        hasHeadroom = true;
    }

    // advance current state and store IL count values
    current_state = next_state;
    cycle_count = cur_i;

    return hasHeadroom;
}

void ILReport(int states_executed)
{
    string il_str = String.Format("IL Count: {0}/{1} ({2:0.0}%)",
        Runtime.CurrentInstructionCount,
        Runtime.MaxInstructionCount,
        (Decimal)Runtime.CurrentInstructionCount / Runtime.MaxInstructionCount * 100M);
    Echo(String.Format("States executed: {0}", states_executed));
    Echo(il_str);
}

public void Save()
{
}

public Program()
{
    states = new Action[] {
            s_refreshGrids,
            s_refreshGroups,
            s_processStates,
            s_displayStats,
        };
    Me.CustomName = "STASYS CPU";
    Me.ShowOnHUD = false;
    Me.CustomData = "";
    current_state = 0;
    state_cycle_counts = new int[states.Length];
    control = new STASYS_Control(broadcast, runtime, echo);
    cur_time = new TimeSpan();
}

void Main()
{
    cur_time += Runtime.TimeSinceLastRun;
    Echo(String.Format("STASYS version {0}", VERSION));
    int num_states = 0;
    cycle_count = 0;
    do
    {
        try
        {
            states[current_state]();
        }
        catch (Exception e)
        {
            Me.CustomName = "STASYS Exception";
            Me.ShowOnHUD = true;
            Echo(e.StackTrace);
            throw;
        }
        num_states++;
    } while (canContinue() && num_states < states.Length);
    Echo(String.Format("Groups under control: {0}", control.groupCount()));

    ILReport(num_states);
}
