(deftemplate pddl-action-executor                                                           
" Interface to the execution layer. Asserted when an (action) is ready to be executed by a worker. 
  Once the action has started executing, at-start effects are applied. Throughout the duration of action,
  feedback is received and corresponding effects are applied.                   
  @slot id: id of the executor.                                                 
  @slot worker: id of the worker which could be a robot or a refbox machine. For now, assuming ROBOT1, ROBOT2, ROBOT3 or REFBOX.
  @slot pddl-action-id: the id of the pddl-action to be executed.               
  @slot state: modelled on the states of ros2 action server responses           
   - INIT: initial state before the execution layer is invoked.                 
   - REQUESTED: the execution layer has been requested to execute the action.   
   - ACCEPTED: the execution layer has accepted the request and has started execution.
   - ABORTED: the execution is aborted due to failure.                          
   - CANCELLED: the execution of action is deliberately cancelled by the user.   
   - SUCCEEDED: the execution layer has succeeded in the execution of the action. 
"                                                                               
  (slot id (type SYMBOL))                                                       
  (slot worker (type SYMBOL))                                                   
  (slot action (type SYMBOL))                                           
  (slot state (type SYMBOL) (allowed-values INIT REQUESTED ACCEPTED ABORTED CANCELLED SUCCEEDED) (default INIT))
)

;(deftemplate 