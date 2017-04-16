# ====================================================================================
# MacroPlannery.py
#   Wrapping the macro planning process.
# ====================================================================================

# =================================================
# PHASE I: scan for transition configurations
# =================================================

with env:
    
    # instantitate single mode sampler
    # TODO: make mode binary so analytic
    query = [np.array([-5, 0, 20.3]), np.array([5, 0, 20.3])]
    sampler0 =  Sampler(mode=0)
    sampler1 =  Sampler(mode=1)
    sampler2 =  Sampler(mode=2)
    modal_samplers = [sampler0, sampler1, sampler2]

    # instantitate multi-modal sampler
    n = float(robots.instance_number-1)
    pr = (float(RADIUS*2)*(n-2))/100
    sampler01 = Sampler(mode=3, is_trans=True, pair=(0,1), pair_range=pr)
    sampler02 = Sampler(mode=4, is_trans=True, pair=(0,2), pair_range=pr)
    sampler12 = Sampler(mode=5, is_trans=True, pair=(1,2), pair_range=pr)
    trans_samplers = [sampler01, sampler02, sampler12]

    # multiModalPlanning
    rtn_tbl, init_node, goal_node = multiModalPlanner(query, robots, modal_samplers, trans_samplers)

    # demo samples
    print 'samples ...'
    print '-'*20
    node = goal_node
    while node!=None:
        print node.getVal()
        node = rtn_tbl[node]

    # collect transition pair
    node = goal_node
    anchors = [goal_node]
    while node!=None:

        # append init node
        if rtn_tbl[node] == None:
            anchors.append(node)
        
        # append anchors with transition configs
        if node.trans_pair != None:
            
            # heuristicly pruning transition to single mode config
            # TODO: use joint state instead of base-represented to prune
            # TODO: explaining transition pruning
            # TODO: not pruning mode-0-to-1 or 2
            if node.trans_pair.getVal()[0]*node.getVal()[0] < 0:
                anchors.append(node)
                anchors.append(node.trans_pair)
                node = node.trans_pair
        node = rtn_tbl[node]

    # demo anchors
    anchors = anchors[::-1]
    print '\nanchors ...'
    print '-'*20
    for v in anchors:
        print v.getVal().tolist()
    
raw_input("Press enter to exit...")

# =================================================
# PHASE II: cost evaluation and heuristic planning
# =================================================

# plan for each transition
for i in range(len(anchors)-1):
    
    # Get init,goal configuratino pair
    init_anchor,goal_anchor = anchors[i:i+2]
    query = [init_anchor.getVal(), goal_anchor.getVal()]
    mode0 = init_anchor.mode
    mode1 = goal_anchor.mode
    print '\nfrom mode%d to mode%d'%(mode0,mode1), '\nquery: ', init_anchor.getVal().tolist(), goal_anchor.getVal().tolist()
    print '-'*20

    query_pair = query
    query = query[1]

    # Planner for section
    if mode0==mode1:
        with env:
            if mode0==UNLOCK:
                robots.setPlanner(astarPlanner, query)
            elif mode0==LOCK0 or LOCKN:
                robots.setPlanner(rotationPlanner, query)
            else:
                pass
            robots.planning()
        robots.release()
        raw_input("Press enter to exit...")

    # Planner for transition
    else:
        with env:
            if mode1==UNLOCK:
                robots.unlock()
            elif mode0==LOCK0:
                robots.lock( LOCK_ROBOT_TEMPLATE, LOCK0 )
            elif mode0==LOCKN:
                robots.lock( LOCK_ROBOT_TEMPLATE, LOCKN )
            robots.planning()
        robots.release()
        raw_input("Press enter to exit...")
