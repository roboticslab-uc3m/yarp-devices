# CanBusControlboard

This device offers a custom way of interfacing with wrapped raw subdevices through [`yarp::dev::IRemoteVariables`](http://www.yarp.it/devel/classyarp_1_1dev_1_1IRemoteVariables.html). The exact output of the following command examples depends on the implementation of the subdevice. Variables `linInterp` and `csv` presented in the examples below are provided by the `TechnosoftIpos` device and parsed in the corresponding implementation of the `yarp::dev::IRemoteVariablesRaw` interface. Here, `linInterp` expects a dictionary (a list of lists), and `csv` parses a single-key-value-paired list.

---

**`getRemoteVariablesList`**

Lists all CAN node IDs prepended with "ID", e.g. `(ID15 ID16 ...)`.

* RPC sample usage: `[get] [ivar] [lvar]`
* Response: `(ID15 ID16 ID17 ID18 ID19 ID20)`

---

**`getRemoteVariable`**

Retrieves all available key-value pairs for the selected node. Calls `getRemoteVariablesListRaw` to get raw keys and then iterates on `getRemoteVariableRaw`.

* RPC sample usage: `[get] [ivar] [mvar] ID15`
* Response: `(ID15 (linInterp ((enable 0))) (csv (enable 0)))`

If `key` equals `all`, it returns remote vars for all available CAN nodes.

* RPC sample usage: `[get] [ivar] [mvar] all`
* Response: `((ID15 (linInterp ((enable 0))) (csv (enable 0))) (ID16 (linInterp ((enable 0))) (csv (enable 0))) (ID17 (linInterp ((enable 0))) (csv (enable 0))) (ID18 (linInterp ((enable 0))) (csv (enable 0))) (ID19 (linInterp ((enable 0))) (csv (enable 0))) (ID20 (linInterp ((enable 0))) (csv (enable 0))))`

---

**`setRemoteVariable`**

Requires a key-value two-element bottle, value is a nested list.

* RPC sample usage: `[set] [ivar] [mvar] ID15 (linInterp ((enable 1) (mode pt) (periodMs 50)))`

Multiple keys can be bound per call, just nest them within an additional layer of bottles.

* RPC sample usage: `[set] [ivar] [mvar] ID15 ((linInterp ((enable 1) (mode pt) (periodMs 50))) (csv (enable 1)))`

Same strategy works if you want to set the exact same remote variable (or variables) for all available nodes, just use `all` as key.

* RPC sample usage: `[set] [ivar] [mvar] all (linInterp ((enable 1) (mode pt) (periodMs 50)))`
* RPC sample usage: `[set] [ivar] [mvar] all ((linInterp ((enable 1) (mode pt) (periodMs 50))) (csv (enable 1)))`

It is also possible to target multiple nodes in a single call, and set one or more remote variables at once. Use `multi` as key.

* RPC sample usage: `[set] [ivar] [mvar] multi ((ID15 (csv (enable 1))) (ID17 (csv (enable 0))))`
* RPC sample usage: `[set] [ivar] [mvar] multi ((ID16 ((linInterp ((enable 0))) (csv (enable 1)))) (ID20 (csv (enable 1))))`
