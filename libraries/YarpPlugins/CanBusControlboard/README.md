# CanBusControlboard

This device offers a custom way of interfacing with wrapped raw subdevices through [`yarp::dev::IRemoteVariables`](http://www.yarp.it/devel/classyarp_1_1dev_1_1IRemoteVariables.html). The exact output of the following command examples depends on the implementation of the subdevice. Variables `linInterp` and `csv` presented in the examples below are provided by the `TechnosoftIpos` device and parsed in the corresponding implementation of the `yarp::dev::IRemoteVariablesRaw` interface. Here, `linInterp` expects a dictionary (a list of lists), and `csv` parses a single-key-value-paired list.

---

**`getRemoteVariablesList`**

Lists all CAN node IDs prepended with "id", e.g. `(id15 id16 ...)`.

* RPC sample usage: `[get] [ivar] [lvar]`
* Response: `(id15 id16 id17 id18 id19 id20)`

---

**`getRemoteVariable`**

Retrieves all available key-value pairs for the selected node. Calls `getRemoteVariablesListRaw` to get raw keys and then iterates on `getRemoteVariableRaw`.

* RPC sample usage: `[get] [ivar] [mvar] id15`
* Response: `(id15 (linInterp ((enable 0))) (csv (enable 0)))`

If `key` equals `all`, it returns remote vars for all available CAN nodes.

* RPC sample usage: `[get] [ivar] [mvar] all`
* Response: `((id15 (linInterp ((enable 0))) (csv (enable 0))) (id16 (linInterp ((enable 0))) (csv (enable 0))) (id17 (linInterp ((enable 0))) (csv (enable 0))) (id18 (linInterp ((enable 0))) (csv (enable 0))) (id19 (linInterp ((enable 0))) (csv (enable 0))) (id20 (linInterp ((enable 0))) (csv (enable 0))))`

---

**`setRemoteVariable`**

Requires a key-value two-element bottle, value is a nested list.

* RPC sample usage: `[set] [ivar] [mvar] id15 (linInterp ((enable 1) (mode pt) (periodMs 50)))`

Multiple keys can be bound per call, just nest them within an additional layer of bottles.

* RPC sample usage: `[set] [ivar] [mvar] id15 ((linInterp ((enable 1) (mode pt) (periodMs 50))) (csv (enable 1)))`

Same strategy works if you want to set the exact same remote variable (or variables) for all available nodes, just use `all` as key.

* RPC sample usage: `[set] [ivar] [mvar] all (linInterp ((enable 1) (mode pt) (periodMs 50)))`
* RPC sample usage: `[set] [ivar] [mvar] all ((linInterp ((enable 1) (mode pt) (periodMs 50))) (csv (enable 1)))`

It is also possible to target multiple nodes in a single call, and set one or more remote variables at once. Use `multi` as key.

* RPC sample usage: `[set] [ivar] [mvar] multi ((id15 (csv (enable 1))) (id17 (csv (enable 0))))`
* RPC sample usage: `[set] [ivar] [mvar] multi ((id16 ((linInterp ((enable 0))) (csv (enable 1)))) (id20 (csv (enable 1))))`
