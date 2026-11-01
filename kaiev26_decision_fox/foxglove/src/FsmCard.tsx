import { ReactElement } from "react";

type FsmCardProps = {
  index: number;
  name: string;
  state: string;
  description: string;
  active: boolean;
};

// Lets a long state name wrap after "_" instead of mid-word in a narrow card.
export function breakable(text: string): string {
  return text.replace(/_/g, "_\u200b");
}

export function FsmCard({ index, name, state, description, active }: FsmCardProps): ReactElement {
  return (
    <article className={`fsm-card ${active ? "active" : ""}`}>
      <span className="fsm-index">{String(index).padStart(2, "0")}</span>
      <div className="fsm-main">
        <h3>{name}</h3>
        <strong>{active ? breakable(state) : "INACTIVE"}</strong>
      </div>
      <p>{active ? description : "판단 대기"}</p>
    </article>
  );
}
