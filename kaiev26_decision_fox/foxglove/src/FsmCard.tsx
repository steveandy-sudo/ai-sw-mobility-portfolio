import { ReactElement } from "react";

type FsmCardProps = {
  index: number;
  name: string;
  state: string;
  description: string;
  active: boolean;
};

export function FsmCard({ index, name, state, description, active }: FsmCardProps): ReactElement {
  return (
    <article className={`fsm-card ${active ? "active" : ""}`}>
      <span className="fsm-index">{String(index).padStart(2, "0")}</span>
      <h3>{name}</h3>
      <strong>{active ? state : "INACTIVE"}</strong>
      <p>{active ? description : "판단 대기"}</p>
      <span className="fsm-status">{active ? "ACTIVE" : "STANDBY"}</span>
    </article>
  );
}
