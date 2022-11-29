###########################################################################
CommonRoad-Reach: A Toolbox for Reachability Analysis of Automated Vehicles
###########################################################################
Reachability analysis has gained considerable popularity in motion planning and safeguarding of
automated vehicles (AVs). While existing tools for reachability analysis mainly focus on
general-purpose algorithms for formal verification of dynamical systems, a toolbox tailored to 
AV-specific applications is not yet available. The CommonRoad-Reach toolbox

- integrates two methods for computing reachable sets, i.e., using polytopic set propagation and graph-based propagation;

- extracts driving corridors which can be used as planning constraints for motion planners; and

- provides Python and C++ implementations of the algorithms, offering convenient prototyping and real-time computation to the users.

.. seealso::
	* `CommonRoad Input-Output <https://commonroad.in.tum.de/commonroad-io>`_
	* `CommonRoad Drivability Checker <https://commonroad.in.tum.de/drivability-checker>`_
	* `CommonRoad Route Planner <https://commonroad.in.tum.de/route-planner>`_
	* `Vehicle Models <https://commonroad.in.tum.de/model-cost-functions>`_

********
Overview
********
.. toctree::
   :maxdepth: 2

   Getting Started <getting_started.rst>
   API Documentation <api/index.rst>
   
.. Add changelog
.. mdinclude:: ../../../CHANGELOG.md

********
Citation
********
.. code-block:: text

   @InProceedings{iraniliu2022commonroad,
      author    = {Irani Liu, Edmond and W\"ursching, Gerald and Klischat, Moritz and Althoff, Matthias},
      booktitle = {Proc. of the IEEE Int. Conf. Intell. Transp. Syst.},
      title     = {{CommonRoad-Reach}: {A} toolbox for reachability analysis of automated vehicles},
      year      = {2022},
      pages     = {1--8}
   }


*******************
Contact Information
*******************
:Website: `http://commonroad.in.tum.de <https://commonroad.in.tum.de/>`_
:Forum: `https://commonroad.in.tum.de/forum/c/comonroad-reach/19 <https://commonroad.in.tum.de/forum/c/comonroad-reach/19>`_
:Email: `commonroad@lists.lrz.de <commonroad@lists.lrz.de>`_

******************
Indices and Tables
******************
* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`