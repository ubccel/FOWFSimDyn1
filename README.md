# FOWFSim-Dyn model

This model is used for simulating the behavior of a 5MW wind farm with users specifying the parameters and input conditions.
<br />
With this model, users can observe the motion of the turbines and power outputs for a 5MW wind farm with yaw angle, blade pitch angle andgenerator torque as the model inputs.

### Important files

Please read 'user_manual.pdf' for detailed instructions.
<br />
<br />
| Files      | Description |
| ----------- | ----------- |
| MainScript.m      | the code where users specify the parameters and inputs.       |
| Simulink_StateDerivVec.slx   | the model where Matlab doing the simulation, users may open this to change the input type and oberseve some intermediate output.       | 


### Reference
-  Ali  C.  Kheirabadi  and  Ryozo  Nagamune.  “A  low-fidelity  dynamic  wind  farm  modelfor simulating time-varying wind conditions and floating platform motion”. In:OceanEngineering234 (2021), p.109313. issn: 0029-8018. doi:https://doi.org/10.1016/j.oceaneng.2021.109313. url:https://www.sciencedirect.com/science/article/pii/S0029801821007290.
