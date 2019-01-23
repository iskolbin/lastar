local astar = {}

function astar.find( getneighbors, heuristics, graph, start, goal )
	local opennode, openheuristics, nopen = {goal}, {0}, 1
	local closedfrom, closedcost = {}, {[goal] = 0}
	local nodebuffer, costbuffer = {}, {}
	while nopen > 0 do
		local currentnode = opennode[1]

		-- We are finding path from the goal to the the start to remove reverse step
		if currentnode == start then
			local path, from, i = {start}, closedfrom[start], 1
			while from ~= nil do
				i = i + 1
				path[i] = from
				from = closedfrom[from]
			end
			return path
		end

		-- Put the last node on the top
		opennode[1], opennode[nopen] = opennode[nopen], nil
		openheuristics[1], openheuristics[nopen] = openheuristics[nopen], nil
		nopen = nopen-1
		-- Heapify by sifting top down
		local i, left, right = 1, 2, 3
		while left <= nopen do
			local smaller = left
			if right <= nopen and openheuristics[left] > openheuristics[right] then
				smaller = right
			end
			if openheuristics[i] > openheuristics[smaller] then
				opennode[i], opennode[smaller] = opennode[smaller], opennode[i]
				openheuristics[i], openheuristics[smaller] = openheuristics[smaller], openheuristics[i]
			else
				break
			end
			i = smaller
			left = i + i
			right = left + 1
		end

		local currentcost = closedcost[currentnode]
		for neighbor, addneighborcost in pairs( getneighbors( graph, currentnode )) do
			local neighborcost = currentcost + addneighborcost
			local storedcost = closedcost[neighbor]
			-- Update closed set if the node is not visited or has worse cost
			if storedcost == nil or storedcost > neighborcost then
				closedcost[neighbor] = neighborcost
				closedfrom[neighbor] = currentnode
				-- Put new node in the the end of the heap
				nopen = nopen + 1
				opennode[nopen] = neighbor
				openheuristics[nopen] = neighborcost + heuristics( graph, neighbor, goal )
				-- Heapify by sifting new node up
				local k, parent = nopen, nopen/2
				parent = parent - parent%1 -- Hacky floor, faster on vanilla Lua
				while k > 1 and openheuristics[parent] > openheuristics[k] do
					opennode[parent], opennode[k] = opennode[k], opennode[parent]
					openheuristics[parent], openheuristics[k] = openheuristics[k], openheuristics[parent]
					k, parent = parent, parent/2
					parent = parent - parent%1
				end
			end
		end
	end
end

return astar
