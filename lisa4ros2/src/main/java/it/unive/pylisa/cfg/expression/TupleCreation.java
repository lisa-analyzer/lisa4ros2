package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.NaryExpression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import org.apache.commons.lang3.StringUtils;

public class TupleCreation extends NaryExpression {

	public TupleCreation(
			CFG cfg,
			CodeLocation loc,
			Expression... values) {
		super(cfg, loc, "tuple", values);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public String toString() {
		Expression[] subs = getSubExpressions();
		if (subs.length == 1)
			return "(" + StringUtils.join(subs, ", ") + ")";
		return super.toString();
	}

	@Override
	public <A extends AbstractState<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		// a tuple creation is also created when parsing expressions between
		// parentheses
		// if we only have one sub-expression, we assume to be in that case
		if (params.length == 1)
			return state;

		AnalysisState<A> result = state.bottom();
		Type tupleType = PyClassType.lookup(LibrarySpecificationProvider.TUPLE);

		// allocate the heap region
		MemoryAllocation alloc = new MemoryAllocation(tupleType, getLocation());
		AnalysisState<A> sem = state.smallStepSemantics(alloc, this);

		// assign the pairs
		AnalysisState<A> assign = state.bottom();
		for (SymbolicExpression loc : sem.getComputedExpressions()) {
			HeapReference ref = new HeapReference(tupleType, loc, getLocation());
			HeapDereference deref = new HeapDereference(tupleType, ref, getLocation());

			for (int i = 0; i < params.length; i++) {
				AnalysisState<A> fieldResult = state.bottom();
				Constant idx = new Constant(Int32Type.INSTANCE, i, getLocation());
				AccessChild fieldAcc = new AccessChild(Untyped.INSTANCE, deref, idx, getLocation());
				for (SymbolicExpression init : params[i]) {
					AnalysisState<A> fieldState = sem.smallStepSemantics(fieldAcc, this);
					for (SymbolicExpression lenId : fieldState.getComputedExpressions())
						fieldResult = fieldResult.lub(fieldState.assign(lenId, init, this));
				}
				assign = assign.lub(fieldResult);
			}

			// we leave the reference on the stack
			result = result.lub(assign.smallStepSemantics(ref, this));
		}

		return result;
	}
}
