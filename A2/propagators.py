#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete problem solution.  

'''This file will contain different constraint propagators to be used within 
   bt_search.

   propagator == a function with the following template
      propagator(csp, newly_instantiated_variable=None)
           ==> returns (True/False, [(Variable, Value), (Variable, Value) ...]

      csp is a CSP object---the propagator can use this to get access
      to the variables and constraints of the problem. The assigned variables
      can be accessed via methods, the values assigned can also be accessed.

      newly_instaniated_variable is an optional argument.
      if newly_instantiated_variable is not None:
          then newly_instantiated_variable is the most
           recently assigned variable of the search.
      else:
          progator is called before any assignments are made
          in which case it must decide what processing to do
           prior to any variables being assigned. SEE BELOW

       The propagator returns True/False and a list of (Variable, Value) pairs.
       Return is False if a deadend has been detected by the propagator.
       in this case bt_search will backtrack
       return is true if we can continue.

      The list of variable values pairs are all of the values
      the propagator pruned (using the variable's prune_value method). 
      bt_search NEEDS to know this in order to correctly restore these 
      values when it undoes a variable assignment.

      NOTE propagator SHOULD NOT prune a value that has already been 
      pruned! Nor should it prune a value twice

      PROPAGATOR called with newly_instantiated_variable = None
      PROCESSING REQUIRED:
        for plain backtracking (where we only check fully instantiated 
        constraints) 
        we do nothing...return true, []

        for forward checking (where we only check constraints with one
        remaining variable)
        we look for unary constraints of the csp (constraints whose scope 
        contains only one variable) and we forward_check these constraints.

        for gac we establish initial GAC by initializing the GAC queue
        with all constaints of the csp


      PROPAGATOR called with newly_instantiated_variable = a variable V
      PROCESSING REQUIRED:
         for plain backtracking we check all constraints with V (see csp method
         get_cons_with_var) that are fully assigned.

         for forward checking we forward check all constraints with V
         that have one unassigned variable left

         for gac we initialize the GAC queue with all constraints containing V.
   '''

def prop_BT(csp, newVar=None):
    '''Do plain backtracking propagation. That is, do no 
    propagation at all. Just check fully instantiated constraints'''
    
    if not newVar:
        return True, []
    for c in csp.get_cons_with_var(newVar):
        if c.get_n_unasgn() == 0:
            vals = []
            vars = c.get_scope()
            for var in vars:
                vals.append(var.get_assigned_value())
            if not c.check(vals):
                return False, []
    return True, []

def prop_FC(csp, newVar=None):
    '''Do forward checking. That is check constraints with 
       only one uninstantiated variable. Remember to keep 
       track of all pruned variable,value pairs and return '''

    # limit the constraints to ones which have newVar in their scope if needed
    if newVar:
        constraints = csp.get_cons_with_var(newVar)
    else:
        constraints = csp.get_all_cons()

    dwo = False
    pruned_values = []

    for c in constraints:
        # check all the constraints that have only one unassigned variable
        if c.get_n_unasgn() == 1:
           dwo, pruned_values = fc_check(c, c.get_unasgn_vars()[0])
           if dwo:
               return False, pruned_values

    return True, pruned_values


def fc_check(constraint, x):
    pruned_values = []
    for d in x.cur_domain():
        # check if the pair x,d is impossible, if so then prune d from domain
        if not constraint.has_support(x, d):
            x.prune_value(d)
            pruned_values.append((x, d))
    # if there's nothing left in the domain then return a dwo
    if not x.cur_domain():
        return True, pruned_values
    return False, pruned_values


def prop_GAC(csp, newVar=None):
    '''Do GAC propagation. If newVar is None we do initial GAC enforce 
       processing all constraints. Otherwise we do GAC enforce with
       constraints containing newVar on GAC Queue'''
    
    # limit the constraints to ones which have newVar in their scope if needed
    if newVar:
        constraints = csp.get_cons_with_var(newVar)
    else:
        constraints = csp.get_all_cons()

    dwo = False
    pruned_values = []

    # call GAC enforce on the constraints
    dwo,pruned_values = gac_enforce(csp,constraints)

    # if there was a domain wipeout, return false, otherwise true
    if dwo:
        return False, pruned_values
    return True,pruned_values


def gac_enforce(csp,queue):
    pruned_values = []

    # while we have constraints left to check for consistency
    while queue:
        c = queue.pop()
        for v in c.get_scope():
            for d in v.cur_domain():

                # check if there are no satisfying assignments for v,d, then prune d
                if not c.has_support(v,d):
                    v.prune_value(d)
                    pruned_values.append((v,d))
                    if not v.cur_domain():
                        queue = []
                        # if a domain is empty then return a dwo
                        return True, pruned_values # dwo occurred
                    else:
                        # append to the queue all the constrains that aren't in queue and have the variable V
                        for cp in [con for con in csp.get_cons_with_var(v) if con not in queue]:
                            queue.append(cp)
    return False, pruned_values

