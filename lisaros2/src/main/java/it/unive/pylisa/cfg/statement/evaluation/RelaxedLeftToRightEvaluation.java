package it.unive.pylisa.cfg.statement.evaluation;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.evaluation.EvaluationOrder;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;

public class RelaxedLeftToRightEvaluation implements EvaluationOrder {
    public static final RelaxedLeftToRightEvaluation INSTANCE = new RelaxedLeftToRightEvaluation();

    private RelaxedLeftToRightEvaluation() {
    }

    @Override
    public <A extends AbstractState<A>> AnalysisState<A> evaluate(
            Expression[] subExpressions,
            AnalysisState<A> entryState,
            InterproceduralAnalysis<A> interprocedural,
            StatementStore<A> expressions,
            ExpressionSet[] computed)
            throws SemanticException {
        if (subExpressions.length == 0)
            return entryState;

        AnalysisState<A> postState = entryState;
        for (int i = 0; i < computed.length; i++) {
            AnalysisState<A> tmp = subExpressions[i].forwardSemantics(postState, interprocedural, expressions);
            expressions.put(subExpressions[i], tmp);
            computed[i] = tmp.getComputedExpressions();
            if (!tmp.getState().equals(tmp.getState().bottom())) // [WARN] THIS IS UNSOUND!!!!
                postState = tmp;
        }

        return postState;
    }

    @Override
    public <A extends AbstractState<A>> AnalysisState<A> bwdEvaluate(
            Expression[] subExpressions,
            AnalysisState<A> entryState,
            InterproceduralAnalysis<A> interprocedural,
            StatementStore<A> expressions,
            ExpressionSet[] computed)
            throws SemanticException {
        if (subExpressions.length == 0)
            return entryState;

        AnalysisState<A> postState = entryState;
        for (int i = computed.length - 1; i >= 0; i--) {
            AnalysisState<A> tmp = subExpressions[i].backwardSemantics(postState, interprocedural, expressions);
            expressions.put(subExpressions[i], tmp);
            computed[i] = tmp.getComputedExpressions();
            postState = tmp;
        }

        return postState;
    }

    @Override
    public int previous(
            int pos,
            int len) {
        return pos - 1;
    }

    @Override
    public int next(
            int pos,
            int len) {
        return pos == len - 1 ? -1 : pos + 1;
    }

    @Override
    public int first(
            int len) {
        return 0;
    }

    @Override
    public int last(
            int len) {
        return len - 1;
    }

}
