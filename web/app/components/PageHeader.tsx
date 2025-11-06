import React from "react";

type PageHeaderProps = {
  title: string;
};

export function PageHeader(props: PageHeaderProps) {
  return (
    <header className="bg-orange-500 text-white">
      <div className="mx-auto max-w-5xl px-6 py-6">
        <h1 className="text-3xl font-bold tracking-tight">{props.title}</h1>
      </div>
    </header>
  );
}
