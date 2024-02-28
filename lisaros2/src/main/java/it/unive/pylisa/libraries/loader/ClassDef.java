package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.pylisa.cfg.type.PyClassType;
import java.util.Collection;
import java.util.HashSet;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

public class ClassDef {
	private final boolean root;
	private final boolean sealed;
	private final String name;
	private final String base;
	private final Collection<Method> methods = new HashSet<>();
	private final Collection<Field> fields = new HashSet<>();

	public ClassDef(
			boolean root,
			boolean sealed,
			String name,
			String base) {
		this.root = root;
		this.sealed = sealed;
		this.name = name;
		this.base = base;
	}

	public boolean isRoot() {
		return root;
	}

	public boolean isSealed() {
		return sealed;
	}

	public String getName() {
		return name;
	}

	public String getBase() {
		return base;
	}

	public Collection<Method> getMethods() {
		return methods;
	}

	public Collection<Field> getFields() {
		return fields;
	}

	@Override
	public int hashCode() {
		return Objects.hash(fields, methods, name, root, sealed, base);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		ClassDef other = (ClassDef) obj;
		return Objects.equals(fields, other.fields) && Objects.equals(methods, other.methods)
				&& Objects.equals(name, other.name) && root == other.root && sealed == other.sealed
				&& Objects.equals(base, other.base);
	}

	@Override
	public String toString() {
		return "ClassDef [root=" + root + ", sealed=" + sealed + ", name=" + name + ", sup=" + base + ", methods="
				+ methods + ", fields=" + fields + "]";
	}

	public ClassUnit toLiSAUnit(
			CodeLocation location,
			Program program,
			AtomicReference<CompilationUnit> rootHolder) {
		ClassUnit unit = new ClassUnit(location, program, name, this.sealed);
		if (this.root)
			if (rootHolder.get() != null)
				throw new IllegalStateException("More than one root class defined as hierarchy root");
			else
				rootHolder.set(unit);
		return unit;
	}

	public ClassUnit populateUnit(
			CodeLocation location,
			CFG init,
			CompilationUnit root) {
		ClassUnit unit = (ClassUnit) PyClassType.lookup(this.name).getUnit();

		if (this.base != null)
			unit.addAncestor(PyClassType.lookup(this.base).getUnit());
		else if (root != null && unit != root)
			unit.addAncestor(root);

		for (Method mtd : this.methods) {
			NativeCFG construct = mtd.toLiSACfg(location, init, unit);
			if (construct.getDescriptor().isInstance())
				unit.addInstanceCodeMember(construct);
			else
				unit.addCodeMember(construct);
		}

		for (Field fld : this.fields) {
			Global field = fld.toLiSAObject(location, unit);
			if (field.isInstance())
				unit.addInstanceGlobal(field);
			else
				unit.addGlobal(field);
		}

		return unit;
	}
}
