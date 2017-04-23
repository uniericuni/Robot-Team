1. CollectTransitionPoints()
2. HeuristicSearch()

- CollectTransitionPoints using MultiModalPRM
    
    while
        CollectSamples()
        CollectTransitionSamples()
        if isConnect()
            transitionPoints <-- connection transition
        endif
    endwhile

- Pros & Cons

    The transition point detector can be independent from cost evaluation (aka real planner). Thus, the second phase might be redundant in the case that we use both sampling based methods for the 1st and 2nd method, while efficient if we use high-level sensor in the first and optimality in the second.

- Modify Exp

1. Sampler.py
2. config.py
3. full_demo2.py

- [Demo Video](https://youtu.be/1dQyMDnteJ8)
