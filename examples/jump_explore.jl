using JuMP 
using OSQP 
using LinearAlgebra

# model = JuMP.Model(OSQP.Optimizer)
P = 2.5*I(3)
q = ones(3)
G = I(3)
h = 30.0*ones(3)
A = I(3)
b = 4*ones(3)
x_min = zeros(3)
x_max = 5*ones(3) 

try delete(model, model.ext[:ineq])  catch pass; end
try delete(model, model.ext[:eq])  catch pass; end
try delete(model, model.ext[:lims])  catch pass; end

# @variable(model, x[1:3])
@objective(model, Min, 0.5*x'*P*x + q'*x)
model.ext[:ineq] = @constraint(model, G*x .≤ h)
model.ext[:eq] = @constraint(model, A*x .== b)
model.ext[:lims] = @constraint(model, x_min .≤ x .≤ x_max)

optimize!(model)
x_sol = value.(x)