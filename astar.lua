local astar = {}

function astar.find(graph, start_node, goal_node, neighbors_dist, h)
	local open_node, open_h, n_open = {goal_node}, {0}, 1
	local closed_from, closed_cost = {}, {[goal_node] = 0}
	while n_open > 0 do
		local current_node = open_node[1]

		-- We are finding path from the goal_node to the the start_node to remove reverse step
		if current_node == start_node then
			local path, from, i = {start_node}, closed_from[start_node], 1
			while from ~= nil do
				i = i + 1
				path[i] = from
				from = closed_from[from]
			end
			return path
		end

		-- Put the last node on the top
		open_node[1], open_node[n_open] = open_node[n_open], nil
		open_h[1], open_h[n_open] = open_h[n_open], nil
		n_open = n_open-1
		-- Heapify by sifting top down
		local i, left_idx, right_idx = 1, 2, 3
		while left_idx <= n_open do
			local smaller_idx = left_idx
			if right_idx <= n_open and open_h[left_idx] > open_h[right_idx] then
				smaller_idx = right_idx
			end
			if open_h[i] > open_h[smaller_idx] then
				open_node[i], open_node[smaller_idx] = open_node[smaller_idx], open_node[i]
				open_h[i], open_h[smaller_idx] = open_h[smaller_idx], open_h[i]
			else
				break
			end
			i = smaller_idx
			left_idx = i + i
			right_idx = left_idx + 1
		end

		local g = closed_cost[current_node]
		for neighbor, d in neighbors_dist(graph, current_node) do
			local neighbor_cost = g + d
			local stored_cost = closed_cost[neighbor]
			-- Update closed set if the node is not visited or has worse cost
			if stored_cost == nil or stored_cost > neighbor_cost then
				closed_cost[neighbor] = neighbor_cost
				closed_from[neighbor] = current_node
				-- Put new node in the the end of the heap
				n_open = n_open + 1
				open_node[n_open] = neighbor
				open_h[n_open] = neighbor_cost + h(graph, neighbor, goal_node)
				-- Heapify by sifting new node up
				local k, parent_idx = n_open, n_open / 2
				parent_idx = parent_idx - parent_idx%1 -- Hacky floor, faster on vanilla Lua
				while k > 1 and open_h[parent_idx] > open_h[k] do
					open_node[parent_idx], open_node[k] = open_node[k], open_node[parent_idx]
					open_h[parent_idx], open_h[k] = open_h[k], open_h[parent_idx]
					k, parent_idx = parent_idx, parent_idx / 2
					parent_idx = parent_idx - parent_idx%1
				end
			end
		end
	end
end

return astar
