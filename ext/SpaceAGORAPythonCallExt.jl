module SpaceAGORAPythonCallExt

import PythonCall

# Preserve the exact integer-weight Blossom backend for cyclic components.
function select_cyclic!(model, work, selected)
	PythonCall.C.PyGILState_Check() != 0 || throw(ArgumentError(
		"Cyclic interlink matching requires the Python GIL. Run on Python's owning thread or use process workers. " *
		"For threaded calls, release the GIL on its owner and acquire it with PythonCall.GIL.@lock."))
	networkx = try
		PythonCall.pyimport("networkx")
	catch err
		err isa PythonCall.PyException || rethrow()
		throw(ArgumentError(
			"Cyclic interlink matching could not load NetworkX. Install NetworkX in the Python environment " *
			"used by PythonCall, then retry. Python reported: " * sprint(showerror, err)))
	end
	graph = networkx.Graph()
	for edge in work.candidates
		first_vertex, second_vertex = model.graph.edges[edge][1], model.graph.edges[edge][2]
		work.cyclic[first_vertex] || continue
		graph.add_edge(first_vertex, second_vertex;
			weight=PythonCall.pybuiltins.int(string(work.weights[edge])))
	end
	matching = networkx.max_weight_matching(graph; maxcardinality=false)
	# Convert scalar endpoints to avoid PythonCall's nested iterable conversion
	# world-age error on Julia 1.12; the matching and exact weights are unchanged.
	for pair in matching
		first_index = PythonCall.pyconvert(Int, pair[0])
		second_index = PythonCall.pyconvert(Int, pair[1])
		push!(selected, minmax(model.graph.vertices[first_index], model.graph.vertices[second_index]))
	end
	return nothing
end

end # module SpaceAGORAPythonCallExt
