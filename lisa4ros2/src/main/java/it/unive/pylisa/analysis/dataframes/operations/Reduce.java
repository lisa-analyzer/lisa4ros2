package it.unive.pylisa.analysis.dataframes.operations;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.pylisa.analysis.dataframes.operations.selection.DataframeSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.columns.ColumnSelection;
import it.unive.pylisa.analysis.dataframes.operations.selection.rows.RowSelection;
import it.unive.pylisa.symbolic.operators.Enumerations.ReduceKind;

public class Reduce<R extends RowSelection<R>, C extends ColumnSelection<C>> extends DataframeOperation {

	private final ReduceKind type;
	private final DataframeSelection<R, C> selection;

	public Reduce(
			CodeLocation where,
			int index,
			ReduceKind type,
			DataframeSelection<R, C> selection) {
		super(where, index);
		this.type = type;
		this.selection = selection;
	}

	public ReduceKind getType() {
		return type;
	}

	public DataframeSelection<R, C> getSelection() {
		return selection;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = super.hashCode();
		result = prime * result + ((selection == null) ? 0 : selection.hashCode());
		result = prime * result + ((type == null) ? 0 : type.hashCode());
		return result;
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (!super.equals(obj))
			return false;
		if (getClass() != obj.getClass())
			return false;
		Reduce<?, ?> other = (Reduce<?, ?>) obj;
		if (selection == null) {
			if (other.selection != null)
				return false;
		} else if (!selection.equals(other.selection))
			return false;
		if (type != other.type)
			return false;
		return true;
	}

	@Override
	public String toString() {
		return "reshape: " + type + "(" + selection + ")";
	}

	@Override
	protected boolean lessOrEqualSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Reduce<?, ?> o = (Reduce<?, ?>) other;
		if (type != o.type)
			return false;
		return selection.lessOrEqual(o.selection);
	}

	@Override
	protected DataframeOperation lubSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Reduce<?, ?> o = (Reduce<?, ?>) other;
		if (type != o.type)
			return top();
		return new Reduce<>(where, index, type, selection.lub(o.selection));
	}

	@Override
	protected DataframeOperation wideningSameOperation(
			DataframeOperation other)
			throws SemanticException {
		Reduce<?, ?> o = (Reduce<?, ?>) other;
		if (type != o.type)
			return top();
		return new Reduce<>(where, index, type, selection.widening(o.selection));
	}

	@Override
	protected int compareToSameOperation(
			DataframeOperation o) {
		Reduce<?, ?> other = (Reduce<?, ?>) o;
		int cmp = type.compare(other.type);
		if (cmp != 0)
			return cmp;
		return selection.compareTo(other.selection);
	}
}
